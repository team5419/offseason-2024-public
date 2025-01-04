package frc.robot.subsystems.apriltagvision;

import frc.robot.constants.FieldConstants.AprilTagLayoutType;
import java.util.Map;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;

public class AprilTagVisionIOSim implements AprilTagVisionIO {

    public AprilTagVisionIOSim(Supplier<AprilTagLayoutType> layoutSupplier, int index) {}

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void addCamerasToEstimators() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addCamerasToEstimators'");
    }

    @Override
    public Map<String, EstimatedRobotPose> getEstimates() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEstimates'");
    }

    @Override
    public Map<String, EstimatedRobotPose> filterEstimates(Map<String, EstimatedRobotPose> estimates) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'filterEstimates'");
    }

    @Override
    public EstimatedRobotPose fuseEstimate(Map<String, EstimatedRobotPose> estimates) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'fuseEstimate'");
    }
}
