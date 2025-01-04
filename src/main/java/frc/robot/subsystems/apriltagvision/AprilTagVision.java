package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.VecBuilder;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants.AprilTagLayoutType;
import frc.robot.lib.VirtualSubsystem;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class AprilTagVision extends VirtualSubsystem {

    private AprilTagVisionIO[] ios;

    public AprilTagVision(Supplier<AprilTagLayoutType> layoutSupplier, AprilTagVisionIO... io) {
        ios = io;
        if (AprilTagVisionConstants.kUsingVision) {
            for (AprilTagVisionIO visionIO : ios) {
                visionIO.addCamerasToEstimators();
            }
        }
    }

    @Override
    public void periodic() {
        for (AprilTagVisionIO io : ios) {
            if (!AprilTagVisionConstants.kUsingVision) continue;
            Map<String, EstimatedRobotPose> estimates = io.getEstimates();
            Map<String, EstimatedRobotPose> filteredEstimates = io.filterEstimates(estimates);
            for (EstimatedRobotPose e : filteredEstimates.values()) {
                Logger.recordOutput("estimate " + e.toString(), e.estimatedPose);
            }
            double devX = 1.5; // m
            double devY = 1.5; // m
            double devRot = 3.5; // radF

            EstimatedRobotPose fusedEstimate = io.fuseEstimate(filteredEstimates);
            Logger.recordOutput("fusedEstimate ", fusedEstimate.estimatedPose);

            RobotState.getInstance().addVisionObservation(fusedEstimate, VecBuilder.fill(devX, devY, devRot));
        }
    }
}
