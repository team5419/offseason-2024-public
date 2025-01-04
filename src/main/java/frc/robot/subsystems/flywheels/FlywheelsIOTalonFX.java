package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelsConstants.*;

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
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GlobalConstants;

public class FlywheelsIOTalonFX implements FlywheelsIO {

    private TalonFX flywheelsFrontMotor, flywheelsBackMotor;

    private final VelocityVoltage reqVelTorque = new VelocityVoltage(0);
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

    private final Slot0Configs controllerConfig = new Slot0Configs();

    private final StatusSignal<Double> motorFrontPosition;
    private final StatusSignal<Double> motorFrontVelocity;
    private final StatusSignal<Double> motorFrontAppliedVoltage;
    private final StatusSignal<Double> motorFrontSupplyCurrent;
    private final StatusSignal<Double> motorFrontTorqueCurrent;
    private final StatusSignal<Double> motorFrontTempCelsius;

    private final StatusSignal<Double> motorBackPosition;
    private final StatusSignal<Double> motorBackVelocity;
    private final StatusSignal<Double> motorBackAppliedVoltage;
    private final StatusSignal<Double> motorBackSupplyCurrent;
    private final StatusSignal<Double> motorBackTorqueCurrent;
    private final StatusSignal<Double> motorBackTempCelsius;

    public FlywheelsIOTalonFX() {
        flywheelsFrontMotor = new TalonFX(kFlywheelsConfig.frontID(), GlobalConstants.kRIOName);
        flywheelsBackMotor = new TalonFX(kFlywheelsConfig.backID(), GlobalConstants.kRIOName);

        motorFrontPosition = flywheelsFrontMotor.getPosition();
        // In RPS
        motorFrontVelocity = flywheelsFrontMotor.getVelocity();
        motorFrontAppliedVoltage = flywheelsFrontMotor.getMotorVoltage();
        motorFrontSupplyCurrent = flywheelsFrontMotor.getSupplyCurrent();
        motorFrontTorqueCurrent = flywheelsFrontMotor.getTorqueCurrent();
        motorFrontTempCelsius = flywheelsFrontMotor.getDeviceTemp();

        motorBackPosition = flywheelsBackMotor.getPosition();
        // In RPS
        motorBackVelocity = flywheelsBackMotor.getVelocity();
        motorBackAppliedVoltage = flywheelsBackMotor.getMotorVoltage();
        motorBackSupplyCurrent = flywheelsBackMotor.getSupplyCurrent();
        motorBackTorqueCurrent = flywheelsBackMotor.getTorqueCurrent();
        motorBackTempCelsius = flywheelsBackMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                kFlywheelFrequency,
                motorFrontPosition,
                motorFrontVelocity,
                motorFrontAppliedVoltage,
                motorFrontSupplyCurrent,
                motorFrontTorqueCurrent,
                motorFrontTempCelsius);

        BaseStatusSignal.setUpdateFrequencyForAll(
                kFlywheelFrequency,
                motorBackPosition,
                motorBackVelocity,
                motorBackAppliedVoltage,
                motorBackSupplyCurrent,
                motorBackTorqueCurrent,
                motorBackTempCelsius);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = kFlywheelGearRatio;

        controllerConfig.kP = gains.kP();
        controllerConfig.kI = gains.kI();
        controllerConfig.kD = gains.kD();

        flywheelsFrontMotor.getConfigurator().apply(config, 1.0);
        flywheelsBackMotor.getConfigurator().apply(config, 1.0);
        flywheelsFrontMotor.getConfigurator().apply(controllerConfig, 1.0);
        flywheelsBackMotor.getConfigurator().apply(controllerConfig, 1.0);

        flywheelsFrontMotor.setInverted(false);
        flywheelsBackMotor.setInverted(true);

        flywheelsFrontMotor.optimizeBusUtilization(0, 1.0);
        flywheelsBackMotor.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        inputs.frontConnected = inputs.frontConnected = BaseStatusSignal.refreshAll(
                        motorFrontPosition,
                        motorFrontVelocity,
                        motorFrontAppliedVoltage,
                        motorFrontSupplyCurrent,
                        motorFrontTorqueCurrent,
                        motorFrontTempCelsius)
                .isOK();
        ;
        inputs.backConnected = inputs.backConnected = BaseStatusSignal.refreshAll(
                        motorBackPosition,
                        motorBackVelocity,
                        motorBackAppliedVoltage,
                        motorBackSupplyCurrent,
                        motorBackTorqueCurrent,
                        motorBackTempCelsius)
                .isOK();
        ;

        inputs.frontPositionRads = Units.rotationsToRadians(motorFrontPosition.getValueAsDouble());
        inputs.frontVelocityRPS = motorFrontVelocity.getValueAsDouble();
        inputs.frontAppliedVolts = motorFrontAppliedVoltage.getValueAsDouble();
        inputs.frontSupplyCurrentAmps = motorFrontSupplyCurrent.getValueAsDouble();
        inputs.frontTorqueCurrentAmps = motorFrontTorqueCurrent.getValueAsDouble();
        inputs.frontTempCelsius = motorFrontTempCelsius.getValueAsDouble();

        inputs.backPositionRads = Units.rotationsToRadians(motorBackPosition.getValueAsDouble());
        inputs.backVelocityRPS = motorBackVelocity.getValueAsDouble();
        inputs.backAppliedVolts = motorBackAppliedVoltage.getValueAsDouble();
        inputs.backSupplyCurrentAmps = motorBackSupplyCurrent.getValueAsDouble();
        inputs.backTorqueCurrentAmps = motorBackTorqueCurrent.getValueAsDouble();
        inputs.backTempCelsius = motorBackTempCelsius.getValueAsDouble();
    }

    @Override
    public void runVolts(double motorVolts) {
        double clampedMotorVolts = MathUtil.clamp(motorVolts, -12, 12);
        flywheelsFrontMotor.setControl(voltageControl.withOutput(clampedMotorVolts));
        flywheelsBackMotor.setControl(voltageControl.withOutput(clampedMotorVolts));
    }

    @Override
    public void stop() {
        flywheelsFrontMotor.setControl(neutralControl);
        flywheelsBackMotor.setControl(neutralControl);
    }

    @Override
    public void runVelocity(double motorRPS, double motorFeedforward) {
        flywheelsFrontMotor.setControl(reqVelTorque.withVelocity(motorRPS).withFeedForward(motorFeedforward));
        flywheelsBackMotor.setControl(reqVelTorque.withVelocity(motorRPS).withFeedForward(motorFeedforward));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controllerConfig.kP = kP;
        controllerConfig.kI = kI;
        controllerConfig.kD = kD;
        flywheelsFrontMotor.getConfigurator().apply(controllerConfig);
        flywheelsBackMotor.getConfigurator().apply(controllerConfig);
    }

    @Override
    public void runCharacterization(double input) {
        runVolts(input);
    }
}
