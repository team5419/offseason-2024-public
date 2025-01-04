package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants.AprilTagLayoutType;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {

    private List<PhotonCamera> cameras = new ArrayList<>();
    private List<PhotonPoseEstimator> photonEstimators = new ArrayList<>();
    private final String loggingRoot = "AprilTagVision/";

    public AprilTagVisionIOPhoton(Supplier<AprilTagLayoutType> layoutSupplier, int index) {
        addCamerasToEstimators();
    }

    @Override
    public void addCamerasToEstimators() {
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        for (int i = 0; i < AprilTagVisionConstants.kCameraQuantity; i++) {
            cameras.add(new PhotonCamera(AprilTagVisionConstants.kCameraNames[i]));
            photonEstimators.add(new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameras.get(i),
                    AprilTagVisionConstants.kCameraTransform[i]));
            photonEstimators.get(i).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    }

    public Map<String, EstimatedRobotPose> getEstimates() {
        Map<String, EstimatedRobotPose> estimates = new HashMap<>();
        for (int i = 0; i < photonEstimators.size(); i++) {
            PhotonPoseEstimator estimator = photonEstimators.get(i);
            PhotonCamera camera = cameras.get(i);
            Optional<EstimatedRobotPose> maybeEstimation = estimator.update();
            if (maybeEstimation.isPresent()) {
                estimates.put(camera.getName(), maybeEstimation.get());
            }
        }
        return estimates;
    }

    public Map<String, EstimatedRobotPose> filterEstimates(Map<String, EstimatedRobotPose> estimates) {
        Map<String, EstimatedRobotPose> filtered = new HashMap<>();
        for (String cameraName : estimates.keySet()) {
            EstimatedRobotPose estimate = estimates.get(cameraName);

            double totalTags = estimate.targetsUsed.size();
            double totalDistance = 0;
            double totalAmbiguity = 0;
            double totalArea = 0;

            for (PhotonTrackedTarget target : estimate.targetsUsed) {
                totalAmbiguity += target.getPoseAmbiguity();
                totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                totalArea += target.getArea();
            }

            double averageAmbiguity = totalAmbiguity / totalTags;
            double averageDistance = totalDistance / totalTags;
            double averageArea = totalArea / totalTags;

            if (!averageAmbiguityCheck(averageAmbiguity)) continue;

            if (!averageDistanceCheck(averageDistance)) continue;

            if (!averageAreaCheck(averageArea)) continue;

            if (!averageSpeedCheck()) continue;

            filtered.put(cameraName, estimate);
        }
        return filtered;
    }

    public EstimatedRobotPose fuseEstimate(Map<String, EstimatedRobotPose> estimates) {

        double totalX = 0;
        double totalY = 0;
        double totalZ = 0;
        Rotation3d totalRotation = new Rotation3d();
        double numEstimates = estimates.values().size();
        List<EstimatedRobotPose> estimateList = estimates.values().stream().toList();
        for (int i = 0; i < numEstimates; i++) {
            EstimatedRobotPose estimate = estimateList.get(i);
            totalX += estimate.estimatedPose.getX();
            totalY += estimate.estimatedPose.getY();
            totalZ += estimate.estimatedPose.getZ();
            totalRotation.plus(estimate.estimatedPose.getRotation());
        }
        Pose3d fusedPose = new Pose3d(
                totalX / numEstimates, totalY / numEstimates, totalZ / numEstimates, totalRotation.div(numEstimates));
        EstimatedRobotPose fusedEstimate = new EstimatedRobotPose(
                fusedPose,
                estimateList.get(0).timestampSeconds,
                estimateList.get(0).targetsUsed,
                estimateList.get(0).strategy);
        return fusedEstimate;
    }

    double constrainAngle(double x) {
        x = (x + 180) % 360;
        if (x < 0) x += 360;
        return x - 180;
    }

    public boolean averageAmbiguityCheck(double averageAmbiguity) {
        // If average ambiguity is too high then reject
        boolean isAmbiguityTooHigh = averageAmbiguity >= AprilTagVisionConstants.kAmbiguityRejectionLimit;
        if (isAmbiguityTooHigh) {
            Logger.recordOutput(
                    loggingRoot + "/Rejection Reason",
                    "Rejecting on ambiguity, calculated ambiguity is: " + averageAmbiguity);
            return false;
        }
        return true;
    }

    public boolean averageDistanceCheck(double averageDistance) {
        // If average distance if too hight then reject
        if (averageDistance >= AprilTagVisionConstants.kDistanceRejectionLimit) {
            Logger.recordOutput(
                    loggingRoot + "/Rejection Reason",
                    "Rejecting on distance, calculated distance: " + averageDistance);
            return false;
        }
        return true;
    }

    public boolean averageAreaCheck(double averageArea) {
        // If average distance if too hight then reject
        if (averageArea <= AprilTagVisionConstants.kAreaMinimum) {
            Logger.recordOutput(
                    loggingRoot + "/Rejection Reason", "Rejecting on area, calculated area: " + averageArea);
            return false;
        }
        return true;
    }

    public boolean averageSpeedCheck() {
        // if average speed too high we're cooked chat
        ChassisSpeeds speeds = RobotState.getInstance().getChassisSpeeds();
        if (Math.abs(speeds.vxMetersPerSecond) > AprilTagVisionConstants.kMaxXVelocity
                || Math.abs(speeds.vyMetersPerSecond) > AprilTagVisionConstants.kMaxYVelocity
                || Math.abs(speeds.omegaRadiansPerSecond) > AprilTagVisionConstants.kMaxRotationalVelocity) {
            Logger.recordOutput(loggingRoot + "/Rejection Reason", "Rejecting on swerve speed");
            return false;
        }
        return true;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {}
}
