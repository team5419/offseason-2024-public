package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelsConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {

    private FlywheelsIO io;
    private FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

    private static final LoggedTunableNumber velocityRPS = new LoggedTunableNumber("Flywheels/Target Velocity RPS", 20);
    private static final LoggedTunableNumber voltage = new LoggedTunableNumber("Flywheels/Voltage", .5);

    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

    private static final LoggedTunableNumber shootingRPS = new LoggedTunableNumber("Flywheels/ShootingRPS", 70.0);
    private static final LoggedTunableNumber intakingRPS = new LoggedTunableNumber("Flywheels/IntakingRPS", -50.0);
    private static final LoggedTunableNumber ejectingRPS = new LoggedTunableNumber("Flywheels/EjectingRPS", 20.0);

    public enum FlywheelsGoal {
        IDLE(() -> 0.0),
        INTAKE(intakingRPS),
        EJECT(ejectingRPS),
        SHOOT(shootingRPS),
        INTERPOLATE(() -> RobotState.getInstance().getShotParameters().interpolatedShooterSpeed());

        @Getter
        private final DoubleSupplier flywheelsSpeed;

        private FlywheelsGoal(DoubleSupplier flywheelsSpeed) {
            this.flywheelsSpeed = flywheelsSpeed;
        }
    }

    @AutoLogOutput(key = "Flywheels/Flywheels Goal")
    private FlywheelsGoal currentGoal = FlywheelsGoal.IDLE;

    public Flywheels(FlywheelsIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheels", inputs);

        // Check controller
        LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(), kSVA -> ff = new SimpleMotorFeedforward(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);

        if (currentGoal == FlywheelsGoal.IDLE) stop();
        else runVelocity(currentGoal.flywheelsSpeed.getAsDouble());
    }

    public void setGoal(FlywheelsGoal desiredGoal) {
        currentGoal = desiredGoal;
    }

    @AutoLogOutput(key = "Flywheels/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
                (inputs.backVelocityRPS + inputs.frontVelocityRPS) / 2,
                currentGoal.flywheelsSpeed.getAsDouble(),
                kFlywheelsTolerance);
    }

    public void runVelocity(double flywheelsSpeed) {
        io.runVelocity(flywheelsSpeed, ff.calculate(velocityRPS.getAsDouble()));
    }

    public void runVelocity() {
        io.runVelocity(velocityRPS.getAsDouble(), ff.calculate(velocityRPS.getAsDouble()));
    }

    public void stop() {
        io.stop();
    }
}
