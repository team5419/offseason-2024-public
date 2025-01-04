package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.GlobalConstants;
import lombok.Getter;

public class IntakeIOTalonFX implements IntakeIO {

    @Getter
    private TalonFX intakeLeftMotor, intakeRightMotor;

    private final Slot0Configs controllerConfig = new Slot0Configs();

    private final VoltageOut reqVoltage =
            new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
    private final NeutralOut reqNeutral = new NeutralOut().withUpdateFreqHz(0.0);
    private final VelocityVoltage reqVelocity = new VelocityVoltage(0).withUpdateFreqHz(0.0);

    private final StatusSignal<Double> leftMotorPosition;
    private final StatusSignal<Double> leftMotorVelocity;
    private final StatusSignal<Double> leftMotorAppliedVoltage;
    private final StatusSignal<Double> leftMotorSupplyCurrent;
    private final StatusSignal<Double> leftMotorTorqueCurrent;
    private final StatusSignal<Double> leftMotorTempCelsius;

    private final StatusSignal<Double> rightMotorPosition;
    private final StatusSignal<Double> rightMotorVelocity;
    private final StatusSignal<Double> rightMotorAppliedVoltage;
    private final StatusSignal<Double> rightMotorSupplyCurrent;
    private final StatusSignal<Double> rightMotorTorqueCurrent;
    private final StatusSignal<Double> rightMotorTempCelsius;

    public IntakeIOTalonFX() {
        intakeLeftMotor = new TalonFX(kIntakeConfig.leftID(), GlobalConstants.kCANivoreName);
        intakeRightMotor = new TalonFX(kIntakeConfig.rightID(), GlobalConstants.kCANivoreName);

        // Get values for the other double suppliers for both motors
        leftMotorPosition = intakeLeftMotor.getPosition();
        leftMotorVelocity = intakeLeftMotor.getVelocity();
        leftMotorAppliedVoltage = intakeLeftMotor.getMotorVoltage();
        leftMotorSupplyCurrent = intakeLeftMotor.getSupplyCurrent();
        leftMotorTorqueCurrent = intakeLeftMotor.getTorqueCurrent();
        leftMotorTempCelsius = intakeLeftMotor.getDeviceTemp();

        rightMotorPosition = intakeRightMotor.getPosition();
        rightMotorVelocity = intakeRightMotor.getVelocity();
        rightMotorAppliedVoltage = intakeRightMotor.getMotorVoltage();
        rightMotorSupplyCurrent = intakeRightMotor.getSupplyCurrent();
        rightMotorTorqueCurrent = intakeRightMotor.getTorqueCurrent();
        rightMotorTempCelsius = intakeRightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                kIntakeFrequency,
                leftMotorPosition,
                leftMotorVelocity,
                leftMotorAppliedVoltage,
                leftMotorSupplyCurrent,
                leftMotorTorqueCurrent,
                leftMotorTempCelsius);

        BaseStatusSignal.setUpdateFrequencyForAll(
                kIntakeFrequency,
                rightMotorPosition,
                rightMotorVelocity,
                rightMotorAppliedVoltage,
                rightMotorSupplyCurrent,
                rightMotorTorqueCurrent,
                rightMotorTempCelsius);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = kIntakeGearRatio;

        controllerConfig.kP = gains.kP();
        controllerConfig.kI = gains.kI();
        controllerConfig.kD = gains.kD();

        intakeLeftMotor.getConfigurator().apply(config, 1.0);
        intakeRightMotor.getConfigurator().apply(config, 1.0);
        intakeLeftMotor.getConfigurator().apply(controllerConfig, 1.0);
        intakeRightMotor.getConfigurator().apply(controllerConfig, 1.0);

        intakeLeftMotor.setInverted(false);
        intakeRightMotor.setInverted(true);

        intakeLeftMotor.optimizeBusUtilization(0, 1.0);
        intakeRightMotor.optimizeBusUtilization(0, 1);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
                        leftMotorPosition,
                        leftMotorVelocity,
                        leftMotorAppliedVoltage,
                        leftMotorSupplyCurrent,
                        leftMotorTorqueCurrent,
                        leftMotorTempCelsius)
                .isOK();

        inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
                        rightMotorPosition,
                        rightMotorVelocity,
                        rightMotorAppliedVoltage,
                        rightMotorSupplyCurrent,
                        rightMotorTorqueCurrent,
                        rightMotorTempCelsius)
                .isOK();

        inputs.leftMotorPositionRads = leftMotorPosition.getValueAsDouble();
        inputs.leftMotorVelocityRPS = leftMotorVelocity.getValueAsDouble();
        inputs.leftMotorAppliedVolts = leftMotorAppliedVoltage.getValueAsDouble();
        inputs.leftMotorSupplyCurrentAmps = leftMotorSupplyCurrent.getValueAsDouble();
        inputs.leftMotorTorqueCurrentAmps = leftMotorTorqueCurrent.getValueAsDouble();
        inputs.leftMotorTempCelsius = leftMotorTempCelsius.getValueAsDouble();

        inputs.rightMotorPositionRads = rightMotorPosition.getValueAsDouble();
        inputs.rightMotorVelocityRPS = rightMotorVelocity.getValueAsDouble();
        inputs.rightMotorAppliedVolts = rightMotorAppliedVoltage.getValueAsDouble();
        inputs.rightMotorSupplyCurrentAmps = rightMotorSupplyCurrent.getValueAsDouble();
        inputs.rightMotorTorqueCurrentAmps = rightMotorTorqueCurrent.getValueAsDouble();
        inputs.rightMotorTempCelsius = rightMotorTempCelsius.getValueAsDouble();
    }

    @Override
    public void runVolts(double motorVolts) {
        double clampedMotorVolts = MathUtil.clamp(motorVolts, -12.0, 12.0);
        intakeLeftMotor.setControl(reqVoltage.withOutput(clampedMotorVolts));
        intakeRightMotor.setControl(reqVoltage.withOutput(clampedMotorVolts));
    }

    @Override
    public void stop() {
        intakeLeftMotor.setControl(reqNeutral);
        intakeRightMotor.setControl(reqNeutral);
    }

    @Override
    public void runVelocity(double motorRpm, double ff) {
        intakeLeftMotor.setControl(reqVelocity.withVelocity(motorRpm).withFeedForward(ff));
        intakeRightMotor.setControl(reqVelocity.withVelocity(motorRpm).withFeedForward(ff));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controllerConfig.kP = gains.kP();
        controllerConfig.kI = gains.kI();
        controllerConfig.kD = gains.kD();

        intakeLeftMotor.getConfigurator().apply(controllerConfig, 1.0);
        intakeRightMotor.getConfigurator().apply(controllerConfig, 1.0);
    }

    /** Run left motor at voltage */
    public void runCharacterizationmotorOne(double input) {}

    /** Run right motor at voltage */
    public void runCharacterizationmotorTwo(double input) {}
}
