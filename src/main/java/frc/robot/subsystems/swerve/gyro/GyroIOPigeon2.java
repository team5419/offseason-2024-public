package frc.robot.subsystems.swerve.gyro;

import static frc.robot.subsystems.swerve.SwerveConstants.kOdometryFreq;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;
import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 pigeon;

    private final StatusSignal<Double> yawSignal, yawVelocitySignal;

    // for future collision detection?
    private final StatusSignal<Double> accelX, accelY;

    private final Queue<Double> yawPositionQueue;

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(Ports.kPigeonID, GlobalConstants.kCANivoreName);
        yawSignal = pigeon.getYaw();
        yawVelocitySignal = pigeon.getAngularVelocityZWorld();
        accelX = pigeon.getAccelerationX();
        accelY = pigeon.getAccelerationY();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(kOdometryFreq, yawSignal, yawVelocitySignal);
        BaseStatusSignal.setUpdateFrequencyForAll(100, accelX, accelY);

        pigeon.optimizeBusUtilization(0, 1.0);

        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon, yawSignal);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected =
                BaseStatusSignal.refreshAll(yawSignal, yawVelocitySignal).isOK();
        inputs.yawPosition = Rotation2d.fromDegrees(yawSignal.getValue());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocitySignal.getValue());

        double xAccel = 9.81 * accelX.getValue();
        double yAccel = 9.81 * accelY.getValue();
        inputs.linearAccelerationMetersPerSecondSquared = Math.hypot(xAccel, yAccel);

        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawPositionQueue.clear();
    }
}
