package frc.robot.subsystems.detectionvision;

import java.util.*;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DetectionFrame implements LoggableInputs {

    public DetectionFrame(double timestamp, PhotonPipelineResult result, List<PhotonTrackedTarget> targets) {}

    public DetectionFrame() {}

    @Override
    public void toLog(LogTable table) {}

    @Override
    public void fromLog(LogTable table) {}
}
