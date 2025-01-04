package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public class ModuleIOTalonFX implements ModuleIO {

    // hardware
    private final TalonFX driveMotor, turnMotor;
    private final CANcoder absoluteTurnEncoder;

    private final Rotation2d absoluteOffset;

    // configs
    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    // control requests
    private final NeutralOut reqNeutralOut = new NeutralOut().withUpdateFreqHz(0);
    private final VoltageOut reqVoltage = new VoltageOut(0).withUpdateFreqHz(0);

    //    private final TorqueCurrentFOC mReqCurrent = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    //    private final VelocityTorqueCurrentFOC mReqVelocityTorque =
    //        new VelocityTorqueCurrentFOC(0.0)
    //        .withSlot(0)
    //        .withUpdateFreqHz(0);

    private final VelocityVoltage reqVelocityVoltage =
            new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);

    //    private final PositionTorqueCurrentFOC mReqPositionTorque =
    //        new PositionTorqueCurrentFOC(0.0)
    //            .withSlot(0)
    //            .withUpdateFreqHz(0);

    private final PositionVoltage reqPositionVoltage =
            new PositionVoltage(0.0).withSlot(0).withEnableFOC(true).withUpdateFreqHz(0);

    // status signals
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTorqueCurrent;

    private final StatusSignal<Double> turnPosition;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnSupplyCurrent;
    private final StatusSignal<Double> turnTorqueCurrent;

    private final StatusSignal<Double> encoderAbsPosition;

    // odometry queues
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // brake mode executor
    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

    public ModuleIOTalonFX(ModuleConfig config) {
        driveMotor = new TalonFX(config.driveID(), GlobalConstants.kCANivoreName);
        turnMotor = new TalonFX(config.turnID(), GlobalConstants.kCANivoreName);
        absoluteTurnEncoder = new CANcoder(config.cancoderID(), GlobalConstants.kCANivoreName);
        absoluteOffset = config.absoluteEncoderOffset();

        // configure encoder
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.MagnetOffset = absoluteOffset.getRotations();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // configure drive motor
        driveConfig.Feedback.RotorToSensorRatio = 1.0;
        driveConfig.Feedback.SensorToMechanismRatio =
                SwerveConstants.kModuleConstants.driveReduction() / (2.0 * Math.PI);
        //        mDriveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kModuleConstants.driveReduction();
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        driveConfig.MotorOutput.Inverted = config.driveMotorInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // only need stator limit if not using torque control
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = 80.0;

        // config turn motor
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.FeedbackRemoteSensorID = config.cancoderID();
        turnConfig.Feedback.RotorToSensorRatio = SwerveConstants.kModuleConstants.turnReduction();
        turnConfig.Feedback.SensorToMechanismRatio = 1.0;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
        turnConfig.MotorOutput.Inverted =
                config.turnMotorInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // only need stator limit if not using torque control
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit = 40.0;

        for (int i = 0; i < 4; i++) {
            boolean ok = driveMotor.getConfigurator().apply(driveConfig, 0.1).isOK();
            ok &= turnMotor.getConfigurator().apply(turnConfig, 0.1).isOK();
            ok &= absoluteTurnEncoder
                    .getConfigurator()
                    .apply(encoderConfig, 0.1)
                    .isOK();
            if (ok) break;
        }

        encoderAbsPosition = absoluteTurnEncoder.getAbsolutePosition();

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTorqueCurrent = driveMotor.getTorqueCurrent();

        turnPosition = turnMotor.getPosition();
        turnVelocity = turnMotor.getVelocity();
        turnAppliedVolts = turnMotor.getMotorVoltage();
        turnSupplyCurrent = turnMotor.getSupplyCurrent();
        turnTorqueCurrent = turnMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                SwerveConstants.kOdometryFreq, drivePosition, turnPosition, encoderAbsPosition);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrent,
                turnTorqueCurrent);

        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor, drivePosition);
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnMotor, turnPosition);

        driveMotor.optimizeBusUtilization(0, 1.0);
        turnMotor.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // inputs.hasCurrentControl = true;

        inputs.encoderConnected =
                BaseStatusSignal.refreshAll(encoderAbsPosition).isOK();

        inputs.driveMotorConnected = BaseStatusSignal.refreshAll(
                        drivePosition, driveVelocity, driveAppliedVolts, driveSupplyCurrent, driveTorqueCurrent)
                .isOK();

        inputs.turnMotorConnected = BaseStatusSignal.refreshAll(
                        turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent, turnTorqueCurrent)
                .isOK();

        //        inputs.drivePositionRads = Units.rotationsToRadians(mDrivePosition.getValue());
        //        inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(mDriveVelocity.getValue());
        inputs.drivePositionRads = drivePosition.getValue();
        inputs.driveVelocityRadsPerSec = driveVelocity.getValue();
        inputs.driveAppliedVolts = driveAppliedVolts.getValue();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValue();
        inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValue();

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(encoderAbsPosition.getValue());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValue());
        inputs.turnVelocityRadsPerSec = Units.rotationsToRadians(turnVelocity.getValue());
        inputs.turnAppliedVolts = turnAppliedVolts.getValue();
        inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValue();
        inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValue();

        inputs.odometryDrivePositionsMeters = drivePositionQueue.stream()
                .mapToDouble(value -> value * SwerveConstants.kDriveConfig.wheelRadius())
                .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void runDriveVolts(double volts) {
        driveMotor.setControl(reqVoltage.withOutput(volts));
    }

    @Override
    public void runTurnVolts(double volts) {
        turnMotor.setControl(reqVoltage.withOutput(volts));
    }

    @Override
    public void runCharacterization(double input) {
        driveMotor.setControl(reqVoltage.withOutput(input));
    }

    @Override
    public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedforward) {
        // not currently using feedforward, as it's done onboard the TalonFX
        // if we end up doing torque control, we will want feedforward again probably
        driveMotor.setControl(
                reqVelocityVoltage.withVelocity(velocityRadsPerSec) // .withFeedForward(feedforward)
                );
    }

    @Override
    public void runTurnPositionSetpoint(double angleRads) {
        turnMotor.setControl(reqPositionVoltage.withPosition(Units.radiansToRotations(angleRads)));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV) {
        driveConfig.Slot0.kP = kP;
        driveConfig.Slot0.kI = kI;
        driveConfig.Slot0.kD = kD;
        driveConfig.Slot0.kS = kS;
        driveConfig.Slot0.kV = kV;
        driveMotor.getConfigurator().apply(driveConfig, 0.01);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kI = kI;
        turnConfig.Slot0.kD = kD;
        turnMotor.getConfigurator().apply(turnConfig, 0.01);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        brakeModeExecutor.execute(() -> {
            synchronized (driveConfig) {
                driveConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                driveMotor.getConfigurator().apply(driveConfig, 0.25);
            }
        });
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        brakeModeExecutor.execute(() -> {
            synchronized (turnConfig) {
                turnConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                turnMotor.getConfigurator().apply(turnConfig, 0.25);
            }
        });
    }

    @Override
    public void stop() {
        driveMotor.setControl(reqNeutralOut);
        turnMotor.setControl(reqNeutralOut);
    }
}
