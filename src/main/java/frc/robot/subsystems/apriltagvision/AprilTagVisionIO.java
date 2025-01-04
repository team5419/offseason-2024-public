// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.apriltagvision;

import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

public interface AprilTagVisionIO {

    class AprilTagVisionIOInputs implements LoggableInputs {
        public AprilTagFrame[] frames = new AprilTagFrame[] {};
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
            frames = new AprilTagFrame[frameCount];
            for (int i = 0; i < frameCount; i++) {
                frames[i] = new AprilTagFrame();
                frames[i].fromLog(table.getSubtable("Frame/" + i));
            }
            fps = table.get("Fps", 0);
        }
    }

    void updateInputs(AprilTagVisionIOInputs inputs);

    void addCamerasToEstimators();

    Map<String, EstimatedRobotPose> getEstimates();

    Map<String, EstimatedRobotPose> filterEstimates(Map<String, EstimatedRobotPose> estimates);

    EstimatedRobotPose fuseEstimate(Map<String, EstimatedRobotPose> estimates);
}
