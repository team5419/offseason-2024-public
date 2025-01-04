package frc.robot.subsystems.detectionvision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.detectionvision.DetectionVision.Target;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DetectionVisionIO {
    class ObjectDetectionVisionIOInputs implements LoggableInputs {
        public DetectionFrame[] frames = new DetectionFrame[] {};
        public long fps = 0;

        @Override
        public void toLog(LogTable table) {
            table.put("FrameCount", frames.length);
            for (int i = 0; i < frames.length; i++) {
                frames[i].toLog(table.getSubtable("Frame/" + i));
            }
            table.put("Fps", fps);
        }

        @Override
        public void fromLog(LogTable table) {
            int frameCount = table.get("FrameCount", 0);
            frames = new DetectionFrame[frameCount];
            for (int i = 0; i < frameCount; i++) {
                frames[i] = new DetectionFrame();
                frames[i].fromLog(table.getSubtable("Frame/" + i));
            }
            fps = table.get("Fps", 0);
        }
    }

    default void updateInputs(ObjectDetectionVisionIOInputs inputs) {}

    default void addCamerasToEstimators() {}

    default void handleVision() {}

    List<Target> getTargets();

    Transform3d getBestCameraToTarget(Target target);
}
