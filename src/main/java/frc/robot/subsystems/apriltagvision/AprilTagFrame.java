package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import lombok.Builder;
import lombok.Getter;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@Builder
public class AprilTagFrame implements LoggableInputs {

    @Getter
    private double timestamp;

    @Getter
    private boolean usedMultiTag;

    @Getter
    private Pose3d pose0, pose1;

    @Getter
    private double ambiguity;

    @Getter
    private int[] tags;

    public AprilTagFrame(
            double timestamp, boolean usedMultiTag, Pose3d pose0, Pose3d pose1, double ambiguity, int[] tags) {
        this.timestamp = timestamp;
        this.usedMultiTag = usedMultiTag;
        this.pose0 = pose0;
        this.pose1 = pose1;
        this.ambiguity = ambiguity;
        this.tags = tags;
    }

    public AprilTagFrame() {
        this(-1, false, new Pose3d(), new Pose3d(), -1, new int[] {});
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Timestamp", timestamp);
        table.put("UsedMultiTag", usedMultiTag);
        table.put("Pose0", pose0);
        table.put("Pose1", pose1);
        table.put("Ambiguity", ambiguity);
        table.put("Tags", tags);
    }

    @Override
    public void fromLog(LogTable table) {
        timestamp = table.get("Timestamp", -1.0);
        usedMultiTag = table.get("UsedMultiTag", true);
        pose0 = table.get("Pose0", new Pose3d());
        pose1 = table.get("Pose1", new Pose3d());
        ambiguity = table.get("Ambiguity", ambiguity);
        tags = table.get("Tags", new int[] {});
    }
}
