package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GlobalConstants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PivotIOTalonFX implements PivotIO {

    private TalonFX rightPivot, leftPivot;

    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private Slot0Configs slot0Configs = talonConfig.Slot0;

    private MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0);

    // Status Signals
    private final List<StatusSignal<Double>> internalPositionRotations;
    private final List<StatusSignal<Double>> velocityRPS;
    private final List<StatusSignal<Double>> appliedVoltage;
    private final List<StatusSignal<Double>> supplyCurrent;
    private final List<StatusSignal<Double>> torqueCurrent;
    private final List<StatusSignal<Double>> tempCelsius;

    private final VoltageOut voltageControl =
            new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

    public PivotIOTalonFX() {
        rightPivot = new TalonFX(kPivotConfig.rightID(), GlobalConstants.kRIOName);
        leftPivot = new TalonFX(kPivotConfig.leftID(), GlobalConstants.kRIOName);

        talonConfig.Slot0.kP = gains.kP();
        talonConfig.Slot0.kI = gains.kI();
        talonConfig.Slot0.kD = gains.kD();
        talonConfig.Slot0.kD = gains.kS();
        talonConfig.Slot0.kD = gains.kV();
        talonConfig.Slot0.kD = gains.kA();
        talonConfig.Slot0.kD = gains.kG();

        talonConfig.Feedback.SensorToMechanismRatio = kGearRatio; // TODO: Fix this

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(kMotionMagicCruiseVel);
        talonConfig.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(kMotionMagicAccel);
        // talonConfig.MotionMagic.MotionMagicJerk = Units.degreesToRotations(kMotionMagicJerk);

        talonConfig.CurrentLimits.StatorCurrentLimit = PivotConstants.kStatorCurrentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightPivot.getConfigurator().apply(talonConfig);
        leftPivot.getConfigurator().apply(talonConfig);

        rightPivot.setInverted(false);
        leftPivot.setInverted(true);

        internalPositionRotations = List.of(leftPivot.getPosition(), rightPivot.getPosition());
        velocityRPS = List.of(leftPivot.getVelocity(), rightPivot.getVelocity());
        appliedVoltage = List.of(leftPivot.getMotorVoltage(), rightPivot.getMotorVoltage());
        supplyCurrent = List.of(leftPivot.getSupplyCurrent(), rightPivot.getSupplyCurrent());
        torqueCurrent = List.of(leftPivot.getTorqueCurrent(), rightPivot.getTorqueCurrent());
        tempCelsius = List.of(leftPivot.getDeviceTemp(), rightPivot.getDeviceTemp());
        BaseStatusSignal.setUpdateFrequencyForAll(
                kPivotFrequency,
                internalPositionRotations.get(0),
                internalPositionRotations.get(1),
                velocityRPS.get(0),
                velocityRPS.get(1),
                appliedVoltage.get(0),
                appliedVoltage.get(1),
                supplyCurrent.get(0),
                supplyCurrent.get(1),
                torqueCurrent.get(0),
                torqueCurrent.get(1),
                tempCelsius.get(0),
                tempCelsius.get(1));

        // Optimize bus utilization
        // rightPivot.optimizeBusUtilization(0, 1.0);
        // leftPivot.optimizeBusUtilization(0, 1.0);

        leftPivot.setPosition(0);
        rightPivot.setPosition(0);
    }

    public void updateInputs(PivotIOInputs inputs) {
        // LoggedTunableNumber.ifChanged(
        //         hashCode(),
        //         () -> setMotionMagic(kMotionMagicCruiseVel.get(), kMotionMagicAccel.get()),
        //         kMotionMagicCruiseVel,
        //         kMotionMagicCruiseVel);
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
                        internalPositionRotations.get(0),
                        velocityRPS.get(0),
                        appliedVoltage.get(0),
                        supplyCurrent.get(0),
                        torqueCurrent.get(0),
                        tempCelsius.get(0))
                .isOK();

        inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
                        internalPositionRotations.get(1),
                        velocityRPS.get(1),
                        appliedVoltage.get(1),
                        supplyCurrent.get(1),
                        torqueCurrent.get(1),
                        tempCelsius.get(1))
                .isOK();

        inputs.leftPosition =
                Rotation2d.fromRotations(internalPositionRotations.get(0).getValueAsDouble());
        inputs.rightPosition =
                Rotation2d.fromRotations(internalPositionRotations.get(1).getValueAsDouble());

        inputs.velocityRadsPerSecond =
                Units.rotationsToRadians(velocityRPS.get(0).getValue());
        inputs.appliedVolts = appliedVoltage.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.supplyCurrentAmps = supplyCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.torqueCurrentAmps = torqueCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.tempCelsius =
                tempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    }

    /** Sets the arm config, must be called before other methods. */
    // goal parameter is in degrees
    @Override
    public void runSetpoint(double goal, double feedforward) {
        // todo implement feedforward
        rightPivot.setControl(reqMotionMagic.withPosition(Units.degreesToRotations(goal)));
        leftPivot.setControl(reqMotionMagic.withPosition(Units.degreesToRotations(goal)));
        Logger.recordOutput(
                "Pivot/left closed loop target",
                Units.rotationsToDegrees(leftPivot.getClosedLoopReference().getValueAsDouble()));
        Logger.recordOutput(
                "Pivot/right closed loop target",
                Units.rotationsToDegrees(rightPivot.getClosedLoopReference().getValueAsDouble()));
    }

    @Override
    public void runVolts(double volts) {
        rightPivot.setControl(voltageControl.withOutput(volts));
        leftPivot.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        rightPivot.setControl(new NeutralOut());
        leftPivot.setControl(new NeutralOut());
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        rightPivot.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        leftPivot.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        rightPivot.getConfigurator().apply(slot0Configs, 0.01);
        leftPivot.getConfigurator().apply(slot0Configs, 0.01);
    }

    // public void setMotionMagic(double vel, double accel) {
    //     talonConfig.MotionMagic.MotionMagicAcceleration = accel;
    //     talonConfig.MotionMagic.MotionMagicCruiseVelocity = vel;
    //     rightPivot.getConfigurator().apply(talonConfig);
    //     leftPivot.getConfigurator().apply(talonConfig);
    // }
}
