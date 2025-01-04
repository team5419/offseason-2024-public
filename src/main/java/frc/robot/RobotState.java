package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.ShotAngleInterpolator;
import frc.robot.lib.ShotAngleInterpolator.InterpolatedSuperstructureState;
import frc.robot.lib.swerve.ModuleLimits;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.detectionvision.DetectionFrame;
import frc.robot.subsystems.swerve.SwerveConstants;
import java.util.NoSuchElementException;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    public record OdometryObservation(
            SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

    // Shot parameter members
    private InterpolatedSuperstructureState cachedShotParams = null;
    private static final double kPoseBufferSizeSeconds = 2.0;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(kPoseBufferSizeSeconds);

    // Pose estimation
    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();

    @Getter
    private DetectionFrame mostRecentDetectionFrame = new DetectionFrame();

    // Vision standard deviations
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    // Odometry
    private final SwerveDriveKinematics kinematics;
    private SwerveDriveWheelPositions lastWheelPositions = new SwerveDriveWheelPositions(new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    });
    private Rotation2d lastGyroAngle = new Rotation2d();
    private Twist2d robotVelocity = new Twist2d();

    @Getter
    private Twist2d trajectoryVelocity = new Twist2d();

    @Getter
    @Setter
    private Pose2d trajectorySetpoint = new Pose2d();

    @Getter
    @Setter
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private RobotState() {
        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(SwerveConstants.kOdometryStateStdDevs.get(i, 0), 2));
        }
        kinematics = SwerveConstants.kKinematics;
    }

    public void addOdometryObservation(OdometryObservation observation) {

        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();

        // Check gyro connected
        if (observation.gyroAngle != null) {
            // Update dtheta for twist if gyro connected
            twist = new Twist2d(
                    twist.dx,
                    twist.dy,
                    observation.gyroAngle().minus(lastGyroAngle).getRadians());
            lastGyroAngle = observation.gyroAngle();
        }

        // Add twist to odometry pose
        odometryPose = odometryPose.exp(twist);

        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);

        // Calculate diff from last odometry pose and add onto pose estimate
        estimatedPose = estimatedPose.exp(twist);
    }

    public void addVisionObservation(EstimatedRobotPose estimate, Matrix<N3, N1> visionMeasurementStdDevs) {

        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - kPoseBufferSizeSeconds > estimate.timestampSeconds) return;
        } catch (NoSuchElementException ex) {
            return;
        }

        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(estimate.timestampSeconds);
        if (sample.isEmpty()) return;

        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            // r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
            r[i] = 0.1;
        }

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; row++) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }

        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, estimate.estimatedPose.toPose2d());

        // scale transform by visionK
        var kTimesTransform = visionK.times(VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // Recalculate current estimate by applying scaled transform
        // to old estimate then replaying odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public void addDetectionObservation(DetectionFrame frame) {
        mostRecentDetectionFrame = frame;
    }

    public void addVelocityData(Twist2d robotVelocity) {
        cachedShotParams = null;
        this.robotVelocity = robotVelocity;
    }

    public void addTrajectoryVelocityData(Twist2d robotVelocity) {
        trajectoryVelocity = robotVelocity;
    }

    public Rotation2d getAbsoluteYawToPosition(Pose2d target) {
        Translation2d diff = getEstimatedPose().getTranslation().minus(target.getTranslation());
        return diff.getAngle()
                .plus(AllianceFlipUtil.apply(target.getRotation()))
                .plus(Rotation2d.fromDegrees(180));
    }

    /**
     * Get the yaw of the robot required to shoot a note at a pose, with velocity compensation
     * @param target - the target to aim at
     * @param timeToTarget - the time it takes to reach the target
     * @return - the calculated yaw
     * @apiNote This is a expensive function, so be careful where you call it
     */
    public Rotation2d getVelocityCompensatedYaw(Pose2d target) {

        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                getChassisSpeeds(), getEstimatedPose().getRotation());
        Pose2d robot = getEstimatedPose();

        double delta = 0.02;
        double timeToTarget = Util.getTimeToTarget(distanceToSpeaker());

        Logger.recordOutput("Debug/Time to target", timeToTarget);

        double fieldAngle = Math.atan2(
                target.getY() - robot.getY() - speeds.vyMetersPerSecond * (delta + timeToTarget),
                target.getX() - robot.getX() - speeds.vxMetersPerSecond * (delta + timeToTarget));

        return Rotation2d.fromRadians(fieldAngle);
    }

    public InterpolatedSuperstructureState getShotParameters() {
        return ShotAngleInterpolator.getInstance()
                .getVelocityCompensatedSuperstructure(0); // ! no velocity compensation
    }

    public void resetPose(Pose2d initialPose) {
        cachedShotParams = null;
        estimatedPose = initialPose;
        odometryPose = initialPose;
        poseBuffer.clear();
    }

    public void resetGyro(Rotation2d gyroAngle) {
        resetPose(new Pose2d(
                RobotState.getInstance().getEstimatedPose().getTranslation(), AllianceFlipUtil.apply(gyroAngle)));
    }

    public void resetGyro() {
        resetGyro(Rotation2d.fromDegrees(180));
    }

    public ModuleLimits getModuleLimits() {
        // TODO: implement to limit speeds to avoid brownout (if we want)
        return new ModuleLimits(0, 0, 0);
    }

    @AutoLogOutput(key = "RobotState/FieldVelocity")
    public Twist2d getFieldVelocity() {
        Translation2d linearFieldVelocity =
                new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
        return new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
    }

    @AutoLogOutput(key = "RobotState/EstimatedPose")
    public Pose2d getEstimatedPose() {
        Logger.recordOutput("RobotState/Estimated Pose", estimatedPose);
        return estimatedPose;
    }

    @AutoLogOutput(key = "RobotState/OdometryPose")
    public Pose2d getOdometryPose() {
        Logger.recordOutput("RobotState/Odometry Pose", odometryPose);
        return odometryPose;
    }

    @AutoLogOutput(key = "RobotState/DistanceToSpeaker")
    public double distanceToSpeaker() {
        return getEstimatedPose()
                .getTranslation()
                .getDistance(AllianceFlipUtil.apply(FieldConstants.Speaker.kCenterSpeakerOpening.toTranslation2d()));
    }
}
