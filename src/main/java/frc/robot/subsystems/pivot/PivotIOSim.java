package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.GlobalConstants;
import org.littletonrobotics.junction.Logger;

public class PivotIOSim implements PivotIO {

    private final PIDController controller =
            new PIDController(PivotConstants.gains.kP(), PivotConstants.gains.kI(), PivotConstants.gains.kD());

    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(2),
            PivotConstants.kReduction,
            PivotConstants.kEstMoi,
            PivotConstants.kLengthMeters,
            Math.toRadians(PivotConstants.kMinAngle),
            Math.toRadians(PivotConstants.kMaxAngle),
            true,
            Math.toRadians(PivotConstants.kStartingAngle));

    private double appliedVolts = 0.0;

    public PivotIOSim() {
        sim.setState(0.0, 0.0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        sim.update(GlobalConstants.kLooperDT);

        final double dummyMotorTemp = 42.0;
        final double current = sim.getCurrentDrawAmps() / 2.0;

        inputs.rightPosition = Rotation2d.fromRadians(sim.getAngleRads());
        inputs.leftPosition = Rotation2d.fromRadians(sim.getAngleRads());
        inputs.velocityRadsPerSecond = sim.getVelocityRadPerSec();
        inputs.tempCelsius = new double[] {dummyMotorTemp, dummyMotorTemp};
        inputs.supplyCurrentAmps = new double[] {current, current};
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
        Logger.recordOutput("Pivot/sim volts", volts);
    }

    @Override
    public void runSetpoint(double setpointDegrees, double feedforward) {
        double clampRads = MathUtil.clamp(
                Math.toRadians(setpointDegrees),
                Math.toRadians(PivotConstants.kMinAngle),
                Math.toRadians(PivotConstants.kMaxAngle));
        runVolts(controller.calculate(sim.getAngleRads(), clampRads) + feedforward);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }
}
