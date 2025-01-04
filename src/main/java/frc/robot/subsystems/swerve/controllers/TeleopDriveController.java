// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.SwerveConstants.kDriveConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.lib.LoggedTunableNumber;

/**
 * Drive controller for outputting {@link ChassisSpeeds} from driver joysticks.
 */
public class TeleopDriveController {

    public record ControllerFrame(double x, double y, double omega, boolean slowMode) {}

    private static final LoggedTunableNumber controllerDeadband = new LoggedTunableNumber("TeleopDrive/Deadband", 0.1);
    private static final LoggedTunableNumber maxAngularVelocityScalar =
            new LoggedTunableNumber("TeleopDrive/MaxAngularVelocityScalar", 0.65);
    private static final LoggedTunableNumber slowModeMultiplier =
            new LoggedTunableNumber("TeleopDrive/SlowModeMultiplier", 0.25);

    private double controllerX = 0;
    private double controllerY = 0;
    private double controllerOmega = 0;
    private boolean slowMode = false;
    private boolean robotRelative = false;

    /**
     * Accepts new drive input from joysticks.
     *
     * @param x             Desired x velocity scalar, -1..1
     * @param y             Desired y velocity scalar, -1..1
     * @param omega         Desired omega velocity scalar, -1..1
     * @param isRobotRelative Robot relative drive
     */
    public void acceptDriveInput(double x, double y, double omega, boolean isRobotRelative, boolean isSlow) {
        controllerX = x;
        controllerY = y;
        controllerOmega = omega;
        robotRelative = isRobotRelative;
        slowMode = isSlow;
    }

    /**
     * Updates the controller with the currently stored state.
     *
     * @return {@link ChassisSpeeds} with driver's requested speeds.
     */
    public ChassisSpeeds update() {
        Translation2d linearVelocity = calcLinearVelocity(controllerX, controllerY);
        return linearVelocityToChassisSpeeds(linearVelocity);
    }

    /**
     * Checks if we should set brake mode on
     * @return True if slow mode is enabled
     */
    public boolean shouldBrake() {
        return slowMode;
    }

    public static Translation2d calcLinearVelocity(double x, double y) {

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), controllerDeadband.get());
        Rotation2d linearDirection = new Rotation2d(x, y);

        // Square magnitude
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Calculate new linear velocity
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
        return linearVelocity;
    }

    protected ControllerFrame getControllerFrame() {
        return new ControllerFrame(controllerX, controllerY, controllerOmega, slowMode);
    }

    protected ChassisSpeeds linearVelocityToChassisSpeeds(Translation2d linearVelocity) {

        double omega = MathUtil.applyDeadband(controllerOmega, controllerDeadband.get());
        omega = Math.copySign(omega * omega, omega);

        final double maxAngularVelocity = kDriveConfig.maxAngularVelocity() * maxAngularVelocityScalar.get();

        if (robotRelative) {

            return new ChassisSpeeds(
                    linearVelocity.getX() * kDriveConfig.maxLinearVelocity(),
                    linearVelocity.getY() * kDriveConfig.maxLinearVelocity(),
                    omega * maxAngularVelocity);

        } else {

            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
            }

            double calcLinearX = linearVelocity.getX() * kDriveConfig.maxLinearVelocity();
            double calcLinearY = linearVelocity.getY() * kDriveConfig.maxLinearVelocity();
            double calcOmega = omega * maxAngularVelocity;

            if (slowMode) {
                calcLinearX *= slowModeMultiplier.get();
                calcLinearY *= slowModeMultiplier.get();
                calcOmega *= slowModeMultiplier.get();
            }

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    calcLinearX,
                    calcLinearY,
                    calcOmega,
                    RobotState.getInstance().getEstimatedPose().getRotation());
        }
    }
}
