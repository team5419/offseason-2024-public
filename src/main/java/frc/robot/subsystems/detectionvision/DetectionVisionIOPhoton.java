package frc.robot.subsystems.detectionvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotState;
import frc.robot.subsystems.detectionvision.DetectionVision.Target;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DetectionVisionIOPhoton implements DetectionVisionIO {

    private List<PhotonCamera> cameras = new ArrayList<>();
    private final String loggingRoot = "DetectionVision/";

    public DetectionVisionIOPhoton(int index) {
        addCamerasToEstimators();
    }

    @Override
    public void addCamerasToEstimators() {
        for (int i = 0; i < DetectionVisionConstants.kCameraQuantity; i++) {
            cameras.add(new PhotonCamera(DetectionVisionConstants.kCameraNames[i]));
        }
    }

    public List<Target> getTargets() {
        List<PhotonTrackedTarget> photon_targets = List.of();
        for (int i = 0; i < cameras.size(); i++) {
            PhotonCamera camera = cameras.get(i);
            PhotonPipelineResult result = camera.getLatestResult();
            photon_targets.addAll(result.getTargets());
        }
        List<Target> targets = (List<Target>) photon_targets.stream()
                .map((photonTarget) -> new Target(
                        loggingRoot, photonTarget.getPoseAmbiguity(), 0, 0, 0, 0, null, null, 0, null, null));
        return targets;
    }

    public PhotonTrackedTarget toPhotonTrackedTarget(Target target) {
        return new PhotonTrackedTarget(
                target.yaw(),
                target.pitch(),
                target.area(),
                target.skew(),
                target.id(),
                target.pose(),
                target.altPose(),
                target.ambiguity(),
                target.minAreaRectCorners(),
                target.detectedCorners());
    }

    public Transform3d getBestCameraToTarget(Target target) {
        return toPhotonTrackedTarget(target).getBestCameraToTarget();
    }

    @Override
    public void handleVision() {
        for (int i = 0; i < cameras.size(); i++) {
            PhotonCamera camera = cameras.get(i);
            PhotonPipelineResult result = camera.getLatestResult();
            if (!result.hasTargets()) return;

            List<PhotonTrackedTarget> targets = result.getTargets();

            PhotonTrackedTarget best = result.getBestTarget();

            Transform3d bestTransform = best.getBestCameraToTarget();
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
}
