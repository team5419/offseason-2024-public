package frc.robot.subsystems.detectionvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotState;
import frc.robot.lib.VirtualSubsystem;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.TargetCorner;

public class DetectionVision extends VirtualSubsystem {

    public record Target(
            String detectedCameraName,
            double yaw,
            double pitch,
            double area,
            double skew,
            int id,
            Transform3d pose,
            Transform3d altPose,
            double ambiguity,
            List<TargetCorner> minAreaRectCorners,
            List<TargetCorner> detectedCorners) {}

    DetectionVisionIO io;

    public DetectionVision(DetectionVisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        if (!DetectionVisionConstants.kUsingVision) return;
        for (int i = 0; i < DetectionVisionConstants.kCameraQuantity; i++) {
            var maybeTarget = getBestTarget();
            if (maybeTarget.isEmpty()) continue;
            var bestTarget = maybeTarget.get();

            Transform3d bestTransform = io.getBestCameraToTarget(bestTarget);

            Transform3d cameraTransform = DetectionVisionConstants.kCameraTransform[i];
            Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
            Pose2d cameraPose = robotPose.plus(new Transform2d(
                    cameraTransform.getTranslation().toTranslation2d(),
                    cameraTransform.getRotation().toRotation2d()));
            Pose2d objectPose = cameraPose.plus(new Transform2d(
                    bestTransform.getTranslation().toTranslation2d(),
                    bestTransform.getRotation().toRotation2d()));
        }
    }

    public Optional<Target> getBestTarget() {
        return io.getTargets().stream().min(Comparator.comparingDouble(Target::ambiguity));
    }
}
