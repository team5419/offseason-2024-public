package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;

public class Util {

    public static double getTimeToTarget(double dist) {
        // TODO: find new values for a and b
        double a = 0.0302358;
        double b = 0.466889;

        return (a * dist) + b;
    }

    public static Pose2d getFuturePosition(double dt) {

        Pose2d robot = RobotState.getInstance().getEstimatedPose();
        ChassisSpeeds speeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(RobotState.getInstance().getChassisSpeeds(), robot.getRotation());

        double dx = robot.getX() + (speeds.vxMetersPerSecond * dt);
        double dy = robot.getY() + (speeds.vyMetersPerSecond * dt);
        double dtheta = robot.getRotation().getRadians() + (speeds.omegaRadiansPerSecond * dt);

        return new Pose2d(dx, dy, new Rotation2d(dtheta));
    }
}
