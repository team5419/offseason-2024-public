package frc.robot.subsystems.detectionvision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class DetectionVisionConstants {
    public static final boolean kUsingVision = true;

    public static final double kCameraQuantity = 2;
    public static final double kVisionRotationRejectionEpsilon = 5;
    public static final double kVisionPositionRejectionEpsilon = 1;

    // OUT OF DATE
    public static final String[] kCameraNames = {"OV9281_0", "OV9281_1"};

    // OUT OF DATE
    public static final Rotation3d[] kCameraRotations = {
        new Rotation3d(0, Units.degreesToRadians(-40 + 15), Units.degreesToRadians(180)),
        new Rotation3d(0, Units.degreesToRadians(-(90 - 61.875)), Units.degreesToRadians(-10)),
    };

    // OUT OF DATE
    public static final Transform3d[] kCameraTransform = {
        new Transform3d(
                Units.inchesToMeters(-12.592 - 4),
                Units.inchesToMeters(-6.714),
                Units.inchesToMeters(22.203),
                kCameraRotations[0]),
        new Transform3d(
                Units.inchesToMeters(11.335),
                Units.inchesToMeters(11.10),
                Units.inchesToMeters(12.045),
                kCameraRotations[1])
    };

    public static final boolean kScaleStandardDevBasedOnDistanceAndAmbiguity = true;
    public static final double kDistanceScaleFactor = 2;
    public static final double kAmbiguityScaleFactor = 2;

    public static final double kAmbiguityRejectionLimit = 0.5;
    public static final double kDistanceRejectionLimit = 6.1;

    public static final boolean kRejectOnVelocity = false;
    public static final double kMaxXVelocity = 2.5; // m / s
    public static final double kMaxYVelocity = 2.5; // m / s
    public static final double kMaxRotationalVelocity = 0.17; // rad / s - (~ 10 deg / s)
}
