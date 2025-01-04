package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.SwerveConstants.kHeadingControllerConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.constants.GlobalConstants;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.swerve.ModuleLimits;
import frc.robot.lib.util.EqualsUtil;
import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HeadingController {
    private static final LoggedTunableNumber kP =
            new LoggedTunableNumber("HeadingController/kP", kHeadingControllerConstants.kP());
    private static final LoggedTunableNumber kD =
            new LoggedTunableNumber("HeadingController/kD", kHeadingControllerConstants.kD());
    private static final LoggedTunableNumber kTolerance =
            new LoggedTunableNumber("HeadingController/ToleranceDegrees", 1.0);
    private static final LoggedTunableNumber kVelocityMult =
            new LoggedTunableNumber("HeadingController/VelocityMultipler", 0.85);
    private static final LoggedTunableNumber kAccelerationMult =
            new LoggedTunableNumber("HeadingController/AccelerationMultipler", 0.85);

    private ProfiledPIDController pidController;

    private final Supplier<Rotation2d> target;

    public HeadingController(Supplier<Rotation2d> target) {
        this.target = target;

        pidController = new ProfiledPIDController(
                kP.get(), 0.0, kD.get(), new TrapezoidProfile.Constraints(0.0, 0.0), GlobalConstants.kLooperDT);
        pidController.setTolerance(Units.degreesToRadians(kTolerance.get()));
        pidController.enableContinuousInput(-Math.PI, Math.PI);

        pidController.reset(
                RobotState.getInstance().getOdometryPose().getRotation().getRadians(),
                RobotState.getInstance().getFieldVelocity().dtheta);
    }

    public double update() {
        pidController.setPID(kP.get(), 0.0, kD.get());
        pidController.setTolerance(Units.degreesToRadians(kTolerance.get()));

        final ModuleLimits limits = SwerveConstants.kModuleLimits;

        double maxAngularAcceleration = limits.maxDriveAcceleration()
                / SwerveConstants.kDriveConfig.driveBaseRadius()
                * kAccelerationMult.get();

        double maxAngularVelocity =
                limits.maxDriveVelocity() / SwerveConstants.kDriveConfig.driveBaseRadius() * kVelocityMult.get();

        pidController.setConstraints(new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));
        double output = pidController.calculate(
                RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
                target.get().getRadians());

        Logger.recordOutput("Swerve/HeadingController/HeadingError", pidController.getPositionError());
        Logger.recordOutput("Swerve/HeadingController/VelocityError", pidController.getVelocityError());

        return output;
    }

    @AutoLogOutput(key = "Swerve/HeadingController/AtGoal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
                pidController.getGoal().position,
                pidController.getSetpoint().position,
                Units.degreesToRadians(kTolerance.get()));
    }
}
