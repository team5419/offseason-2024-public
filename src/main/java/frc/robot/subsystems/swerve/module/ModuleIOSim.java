package frc.robot.subsystems.swerve.module;

import static frc.robot.subsystems.swerve.SwerveConstants.kDriveConfig;
import static frc.robot.subsystems.swerve.SwerveConstants.kModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveMotorSim =
            new DCMotorSim(DCMotor.getKrakenX60Foc(1), kModuleConstants.driveReduction(), 0.025);
    private final DCMotorSim turnMotorSim =
            new DCMotorSim(DCMotor.getKrakenX60Foc(1), kModuleConstants.turnReduction(), 0.004);

    private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0, GlobalConstants.kLooperDT);
    private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0, GlobalConstants.kLooperDT);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private final Rotation2d turnAbsoluteInitPosition;

    private boolean driveCoast = false;
    private SlewRateLimiter driveVoltsLimiter = new SlewRateLimiter(2.5);

    public ModuleIOSim(ModuleConfig config) {
        turnAbsoluteInitPosition = config.absoluteEncoderOffset();
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        }

        if (driveCoast && DriverStation.isDisabled()) {
            runDriveVolts(driveVoltsLimiter.calculate(driveAppliedVolts));
        } else {
            driveVoltsLimiter.reset(driveAppliedVolts);
        }

        driveMotorSim.update(GlobalConstants.kLooperDT);
        turnMotorSim.update(GlobalConstants.kLooperDT);

        inputs.drivePositionRads = driveMotorSim.getAngularPositionRad();
        inputs.driveVelocityRadsPerSec = driveMotorSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveSupplyCurrentAmps = Math.abs(driveMotorSim.getCurrentDrawAmps());

        inputs.turnAbsolutePosition =
                new Rotation2d(turnMotorSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
        inputs.turnPosition = Rotation2d.fromRadians(turnMotorSim.getAngularPositionRad());
        inputs.turnVelocityRadsPerSec = turnMotorSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnSupplyCurrentAmps = Math.abs(turnMotorSim.getCurrentDrawAmps());

        inputs.odometryDrivePositionsMeters =
                new double[] {driveMotorSim.getAngularPositionRad() * kDriveConfig.wheelRadius()};
        inputs.odometryTurnPositions = new Rotation2d[] {Rotation2d.fromRadians(turnMotorSim.getAngularPositionRad())};
    }

    public void runDriveVolts(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotorSim.setInputVoltage(driveAppliedVolts);
    }

    public void runTurnVolts(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnMotorSim.setInputVoltage(turnAppliedVolts);
    }

    @Override
    public void runCharacterization(double input) {
        runDriveVolts(input);
    }

    @Override
    public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedforward) {
        runDriveVolts(driveFeedback.calculate(driveMotorSim.getAngularVelocityRadPerSec(), velocityRadsPerSec)
                + this.feedforward.calculate(velocityRadsPerSec));
    }

    @Override
    public void runTurnPositionSetpoint(double angleRads) {
        runTurnVolts(turnFeedback.calculate(turnMotorSim.getAngularPositionRad(), angleRads));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV) {
        driveFeedback.setPID(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(kS, kV);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        turnFeedback.setPID(kP, kI, kD);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveCoast = !enable;
    }

    @Override
    public void stop() {
        runDriveVolts(0.0);
        runTurnVolts(0.0);
    }
}
