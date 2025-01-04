package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class PivotVisualizer {
    private final String key;
    private final Mechanism2d viz;
    private final MechanismLigament2d arm;
    private final Translation3d kPivotOrigin = new Translation3d(0.0, 0.0, 0.15);
    private final RobotState robotState = RobotState.getInstance();

    public PivotVisualizer(String key, Color color) {
        this.key = key;
        this.viz = new Mechanism2d(20, 20, new Color8Bit(Color.kBlack));
        MechanismRoot2d root = viz.getRoot("pivot", 2, 0.5);
        this.arm = new MechanismLigament2d("arm", 10.0, 20, 6.0, new Color8Bit(color));
        root.append(this.arm);
    }

    public void update(Rotation2d targetRot) {
        arm.setAngle(targetRot.getDegrees());

        Pose3d pose = new Pose3d(
                new Translation3d(
                        kPivotOrigin.getX() + robotState.getEstimatedPose().getX(),
                        kPivotOrigin.getY() + robotState.getEstimatedPose().getY(),
                        kPivotOrigin.getZ()),
                new Rotation3d(
                        0.0,
                        targetRot.getRadians(),
                        robotState.getEstimatedPose().getRotation().getRadians()));

        // Log outputs for 2D/3D visualization
        Logger.recordOutput("Pivot/Mechanism2d/" + key, viz);
        Logger.recordOutput("Pivot/Mech3d/" + key, pose);
    }
}
