package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.GlobalConstants;

public class IntakeIOSim implements IntakeIO {
    private final FlywheelSim motorOneSim =
            new FlywheelSim(DCMotor.getKrakenX60Foc(1), IntakeConstants.kIntakeGearRatio, 0.003634);
    private final FlywheelSim motorTwoSim =
            new FlywheelSim(DCMotor.getKrakenX60Foc(1), IntakeConstants.kIntakeGearRatio, 0.003634);

    private final PIDController motorOneController =
            new PIDController(IntakeConstants.gains.kP(), IntakeConstants.gains.kI(), IntakeConstants.gains.kD());

    private final PIDController motorTwoController =
            new PIDController(IntakeConstants.gains.kP(), IntakeConstants.gains.kI(), IntakeConstants.gains.kD());

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        motorOneSim.update(GlobalConstants.kLooperDT);
        motorTwoSim.update(GlobalConstants.kLooperDT);

        inputs.leftMotorVelocityRPS = motorOneSim.getAngularVelocityRPM();
        inputs.rightMotorVelocityRPS = motorTwoSim.getAngularVelocityRPM();
    }

    @Override
    public void runVolts(double motorVolts) {
        double clampedMotorVolts = MathUtil.clamp(motorVolts, -12.0, 12.0);

        motorOneSim.setInputVoltage(clampedMotorVolts);
        motorTwoSim.setInputVoltage(clampedMotorVolts);
    }

    @Override
    public void runVelocity(double motorRPS, double ff) {

        double motorOutput = motorOneController.calculate(motorOneSim.getAngularVelocityRPM(), motorRPS) + ff;

        runVolts(motorOutput);

        // TODO: log actual rps values?
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        motorOneController.setPID(kP, kI, kD);
        motorTwoController.setPID(kP, kI, kD);
    }

    @Override
    public void stop() {
        runVolts(0);
    }

    @Override
    public void runCharacterizationLeftMotor(double input) {
        motorOneSim.setInputVoltage(MathUtil.clamp(input, -12.0, 12.0));
    }

    @Override
    public void runCharacterizationRightMotor(double input) {
        motorTwoSim.setInputVoltage(MathUtil.clamp(input, -12.0, 12.0));
    }
}
