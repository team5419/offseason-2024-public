package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.GlobalConstants;
import org.littletonrobotics.junction.Logger;

public class FlywheelsIOSim implements FlywheelsIO {
    private final FlywheelSim motorFrontSim =
            new FlywheelSim(DCMotor.getKrakenX60Foc(1), FlywheelsConstants.kFlywheelGearRatio, 0.003634);

    private final FlywheelSim motorBackSim =
            new FlywheelSim(DCMotor.getKrakenX60Foc(1), FlywheelsConstants.kFlywheelGearRatio, 0.003634);

    private final PIDController motorController = new PIDController(
            FlywheelsConstants.gains.kP(), FlywheelsConstants.gains.kI(), FlywheelsConstants.gains.kD());

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        motorFrontSim.update(GlobalConstants.kLooperDT);
        motorBackSim.update(GlobalConstants.kLooperDT);

        inputs.frontVelocityRPS = motorFrontSim.getAngularVelocityRPM();
        inputs.backVelocityRPS = motorBackSim.getAngularVelocityRPM();
        inputs.frontVelocityRPS = motorFrontSim.getAngularVelocityRPM();
        inputs.backVelocityRPS = motorBackSim.getAngularVelocityRPM();
    }

    @Override
    public void runVolts(double motorVolts) {
        double clampedMotorVolts = MathUtil.clamp(motorVolts, -12.0, 12.0);

        motorFrontSim.setInputVoltage(clampedMotorVolts);
        motorBackSim.setInputVoltage(clampedMotorVolts);
    }

    @Override
    public void runVelocity(double motorRpm, double motorFeedforward) {

        double motorOutput =
                motorController.calculate(motorFrontSim.getAngularVelocityRPM(), motorRpm) + motorFeedforward;
        runVolts(motorOutput);

        Logger.recordOutput("Flywheels/TargetMotorRPM", motorRpm);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        motorController.setPID(kP, kI, kD);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void runCharacterization(double input) {
        motorFrontSim.setInputVoltage(MathUtil.clamp(input, -12.0, 12.0));
        motorBackSim.setInputVoltage(MathUtil.clamp(input, -12.0, 12.0));
    }
}
