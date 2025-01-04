package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTagVisionConstants {

    // public static final Pose3d[] kCameraPoses =
    //         switch (GlobalConstants.getRobotType()) {
    //             case ORBIT -> new Pose3d[] {new Pose3d(), new Pose3d()};

    //             case SIMBOT -> new Pose3d[] {new Pose3d(), new Pose3d()};
    //         };

    public static final boolean kUsingVision = true;

    public static final double kCameraQuantity = 2;
    public static final double kVisionRotationRejectionEpsilon = 5;
    public static final double kVisionPositionRejectionEpsilon = 1;

    public static final String[] kCameraNames = {"OV9281_0", "OV9281_1"};

    // RADIUM ROTATIONS/TRANSFORMS
    // OUT OF DATE
    // public static final Rotation3d[] kCameraRotations = {
    //     new Rotation3d(0, Units.degreesToRadians(-40 + 15), Units.degreesToRadians(180)),
    //     new Rotation3d(0, Units.degreesToRadians(-(90 - 61.875)), Units.degreesToRadians(-10)),
    // };

    // // OUT OF DATE
    // public static final Transform3d[] kCameraTransform = {
    //     new Transform3d(
    //             Units.inchesToMeters(-12.592 - 4),
    //             Units.inchesToMeters(-6.714),
    //             Units.inchesToMeters(22.203),
    //             kCameraRotations[0]),
    //     new Transform3d(
    //             Units.inchesToMeters(11.335),
    //             Units.inchesToMeters(11.10),
    //             Units.inchesToMeters(12.045),
    //             kCameraRotations[1])
    // };
    public static final Rotation3d[] kCameraRotations = {
        new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)),
        new Rotation3d(0, Units.degreesToRadians(-28.278836), 0),
    };

    // OUT OF DATE
    public static final Transform3d[] kCameraTransform = {
        new Transform3d(-0.25, -0.2, 0.217, kCameraRotations[0]),
        new Transform3d(0.309, 0.0, 0.155, kCameraRotations[1])
    };

    public static final boolean kScaleStandardDevBasedOnDistanceAndAmbiguity = true;
    public static final double kDistanceScaleFactor = 2;
    public static final double kAmbiguityScaleFactor = 2;

    public static final double kAmbiguityRejectionLimit = 0.5;
    public static final double kDistanceRejectionLimit = 6.1;
    public static final double kAreaMinimum = 0; // 0-100%

    public static final boolean kRejectOnVelocity = false;
    public static final double kMaxXVelocity = 2.5; // m / s
    public static final double kMaxYVelocity = 2.5; // m / s
    public static final double kMaxRotationalVelocity = 0.17; // rad / s - (~ 10 deg / s)
}
