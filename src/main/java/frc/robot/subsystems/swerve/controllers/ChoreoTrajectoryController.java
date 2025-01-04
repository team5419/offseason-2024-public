package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.GeomUtil;
import java.util.Arrays;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ChoreoTrajectoryController implements TrajectoryController {

    private static final LoggedTunableNumber linearKp =
            new LoggedTunableNumber("Trajectory/linearKp", kTrajectoryConstants.linearKp());
    private static final LoggedTunableNumber linearKd =
            new LoggedTunableNumber("Trajectory/linearKd", kTrajectoryConstants.linearKd());

    private static final LoggedTunableNumber angularKp =
            new LoggedTunableNumber("Trajectory/angularKp", kTrajectoryConstants.thetaKp());
    private static final LoggedTunableNumber angularKd =
            new LoggedTunableNumber("Trajectory/angularKd", kTrajectoryConstants.thetaKd());

    private final ChoreoTrajectory trajectory;
    private final ChoreoControlFunction controlFunc;
    private final Timer timer = new Timer();

    private PIDController xController, yController, thetaController;

    public ChoreoTrajectoryController(ChoreoTrajectory trajectory) {
        this.trajectory = trajectory;

        xController = new PIDController(linearKp.get(), 0.0, linearKd.get());
        yController = new PIDController(linearKp.get(), 0.0, linearKd.get());
        thetaController = new PIDController(angularKp.get(), 0, angularKd.get());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        controlFunc = Choreo.choreoSwerveController(xController, yController, thetaController);

        timer.stop();
        timer.reset();
        timer.start();

        Logger.recordOutput(
                "Trajectory/TrajectoryPoses",
                Arrays.stream(trajectory.getPoses())
                        .map(AllianceFlipUtil::apply)
                        .toArray(Pose2d[]::new));
    }

    @Override
    public ChassisSpeeds update() {
        final ChoreoTrajectoryState sample =
                trajectory.sample(timer.get(), DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

        final Pose2d trajPose = sample.getPose();
        final Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

        ChassisSpeeds output = controlFunc.apply(currentPose, sample);

        final double translationalError = currentPose.getTranslation().getDistance(trajPose.getTranslation());
        final Rotation2d rotationalError = currentPose.getRotation().minus(trajPose.getRotation());

        RobotState.getInstance().setTrajectorySetpoint(trajPose);
        RobotState.getInstance()
                .addTrajectoryVelocityData(ChassisSpeeds.fromFieldRelativeSpeeds(
                                sample.velocityX, sample.velocityY, sample.angularVelocity, trajPose.getRotation())
                        .toTwist2d());

        Logger.recordOutput("Trajectory/SetpointPose", trajPose);
        Logger.recordOutput("Trajectory/SetpointSpeeds/vx", sample.velocityX);
        Logger.recordOutput("Trajectory/SetpointSpeeds/vy", sample.velocityY);
        Logger.recordOutput("Trajectory/SetpointSpeeds/omega", sample.angularVelocity);
        Logger.recordOutput("Trajectory/OutputSpeeds", output);
        Logger.recordOutput("Trajectory/TranslationError", translationalError);
        Logger.recordOutput("Trajectory/RotationError", rotationalError);

        return output;
    }

    @AutoLogOutput(key = "Trajectory/Finished")
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTime());
    }
}
