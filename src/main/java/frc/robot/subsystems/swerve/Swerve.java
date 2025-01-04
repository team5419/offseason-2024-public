package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.kKinematics;
import static frc.robot.subsystems.swerve.SwerveConstants.kModuleLimits;
import static frc.robot.subsystems.swerve.SwerveConstants.kModuleTranslations;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.constants.GlobalConstants;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.swerve.ModuleLimits;
import frc.robot.lib.swerve.SwerveSetpoint;
import frc.robot.lib.swerve.SwerveSetpointGenerator;
import frc.robot.lib.util.GeomUtil;
import frc.robot.subsystems.swerve.controllers.ChoreoTrajectoryController;
import frc.robot.subsystems.swerve.controllers.HeadingController;
import frc.robot.subsystems.swerve.controllers.TeleopDriveController;
import frc.robot.subsystems.swerve.controllers.TrajectoryController;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class Swerve extends SubsystemBase {

    private static final LoggedTunableNumber kCoastWaitTimeSeconds =
            new LoggedTunableNumber("Swerve/CoastWaitTimeSeconds", 0.5);
    private static final LoggedTunableNumber kCoastThresholdMPS =
            new LoggedTunableNumber("Swerve/CoastThresholdMPS", 0.05);

    public enum SwerveMode {
        TELEOP,
        TRAJECTORY,
        AUTO_ALIGN,
        CHARACTERIZATION,
        WHEEL_RADIUS_CHARACTERIZATION
    }

    public enum BrakeModeRequest {
        ALWAYS_BRAKE,
        ALWAYS_COAST,
        AUTO
    }

    @AutoLog
    public static class OdometryTimestampInputs {
        public double[] timestamps = new double[] {};
    }

    // odometry variables
    protected static ReentrantLock odometryLock = new ReentrantLock();
    protected static Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);

    // IO and inputs
    private final OdometryTimestampInputsAutoLogged timestampInputs = new OdometryTimestampInputsAutoLogged();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];

    // state variables
    @AutoLogOutput(key = "Swerve/BrakeModeRequested")
    @Setter
    private BrakeModeRequest brakeModeRequested = BrakeModeRequest.ALWAYS_BRAKE;

    @AutoLogOutput(key = "Swerve/BrakeModeEnabled")
    private boolean brakeModeEnabled = true;

    private SwerveMode currentMode = SwerveMode.TELEOP;
    private boolean lastEnabled = false;
    private Timer lastMovementTimer = new Timer();

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private SwerveDriveWheelPositions lastPositions = null;
    private double lastTime = 0.0;

    private double characterizationInput = 0.0;
    private boolean modulesOrienting = false;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    });

    private final SwerveSetpointGenerator setpointGenerator;

    // controllers
    private TeleopDriveController teleopController;
    private HeadingController headingController;
    private TrajectoryController trajectoryController;

    public Swerve(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(fl, 0);
        modules[1] = new Module(fr, 1);
        modules[2] = new Module(bl, 2);
        modules[3] = new Module(br, 3);

        // make sure the JVM loads these classes
        // this is used to fix delay at start of auto
        // for(int i = 0; i < 25; i++) {
        // TrajectoryController c = new ChoreoTrajectoryController(new
        // ChoreoTrajectory());
        // for(int j = 0; j < 25; j++) {
        // c.update();
        // }
        // }

        RobotState.getInstance().addTrajectoryVelocityData(new Twist2d());

        setpointGenerator = SwerveSetpointGenerator.builder()
                .kinematics(kKinematics)
                .moduleLocations(kModuleTranslations)
                .build();

        teleopController = new TeleopDriveController();
        // teleopController = new AssistedDriveController(); // Use assisted pickup

    }

    @Override
    public void periodic() {

        logOdometryAndGyro();

        final ModuleLimits currentModuleLimits = kModuleLimits;
        updateOdometry(currentModuleLimits);

        updateChassisSpeeds();

        manageSwerveRequests();

        optimizeSwerveModuleSetpoints(currentModuleLimits); // Logs setpoints and torques!

        logRecordOutputsSwerve(); // This name is cooked but its so you can easily find it

        RobotState.getInstance().setChassisSpeeds(getSpeeds());
    }

    private void logOdometryAndGyro() {

        // lock to prevent race on odom data
        odometryLock.lock();
        // collect timestamps from queue so we know exact timestamps of all odom updates since last cycle
        timestampInputs.timestamps =
                timestampQueue.stream().mapToDouble(Double::valueOf).toArray();

        // if there were no updates for some reason, create one
        if (timestampInputs.timestamps.length == 0) {
            timestampInputs.timestamps = new double[] {Timer.getFPGATimestamp()};
        }

        // clear queue so we don't reuse old timestamps
        timestampQueue.clear();

        // log timestamps
        Logger.processInputs("Swerve/OdometryTimestamps", timestampInputs);

        // get updated inputs from gyro and log
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);

        // ensure all modules update
        Arrays.stream(modules).forEach(Module::updateInputs);

        // unlock the mutex, end of critical section
        odometryLock.unlock();
    }

    public void updateOdometry(ModuleLimits currentModuleLimits) {

        // Calculate the min odometry position updates across all modules
        int minOdometryUpdates = IntStream.of(
                        timestampInputs.timestamps.length,
                        Arrays.stream(modules)
                                .mapToInt(module -> module.getModulePositions().length)
                                .min()
                                .orElse(0))
                .min()
                .orElse(0);

        if (gyroInputs.connected) {
            minOdometryUpdates = Math.min(gyroInputs.odometryYawPositions.length, minOdometryUpdates);
        }

        // Pass odometry data to robot state
        for (int i = 0; i < minOdometryUpdates; i++) {

            int odometryIndex = i;

            Rotation2d yaw = gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null;

            // Get all four swerve module positions at that odometry update
            // and store in SwerveDriveWheelPositions object
            SwerveDriveWheelPositions wheelPositions = new SwerveDriveWheelPositions(Arrays.stream(modules)
                    .map(module -> module.getModulePositions()[odometryIndex])
                    .toArray(SwerveModulePosition[]::new));

            // Filtering based on delta wheel positions
            boolean includeMeasurement = true;

            if (lastPositions != null) {
                double dt = timestampInputs.timestamps[i] - lastTime;
                for (int j = 0; j < modules.length; j++) {

                    double velocity =
                            (wheelPositions.positions[j].distanceMeters - lastPositions.positions[j].distanceMeters)
                                    / dt;
                    double omega = wheelPositions
                                    .positions[j]
                                    .angle
                                    .minus(lastPositions.positions[j].angle)
                                    .getRadians()
                            / dt;

                    // Check if delta is too large
                    if (Math.abs(omega) > currentModuleLimits.maxSteeringVelocity() * 5.0
                            || Math.abs(velocity) > currentModuleLimits.maxDriveVelocity() * 5.0) {
                        includeMeasurement = false;
                        break;
                    }
                }
            }

            // If delta isn't too large we can include the measurement.
            if (includeMeasurement) {
                lastPositions = wheelPositions;
                RobotState.getInstance()
                        .addOdometryObservation(
                                new RobotState.OdometryObservation(wheelPositions, yaw, timestampInputs.timestamps[i]));
                lastTime = timestampInputs.timestamps[i];
            }
        }
    }

    private void updateChassisSpeeds() {
        // Update current velocities use gyro when possible
        ChassisSpeeds robotRelativeVelocity = getSpeeds();

        robotRelativeVelocity.omegaRadiansPerSecond =
                gyroInputs.connected ? gyroInputs.yawVelocityRadPerSec : robotRelativeVelocity.omegaRadiansPerSecond;
        RobotState.getInstance().addVelocityData(robotRelativeVelocity.toTwist2d());

        if (Arrays.stream(modules)
                .anyMatch((module) -> Math.abs(module.getVelocityMetersPerSec()) > kCoastThresholdMPS.get())) {
            lastMovementTimer.restart();
        }
    }

    private void manageSwerveRequests() {
        if (!lastEnabled && DriverStation.isEnabled()) {
            brakeModeRequested = BrakeModeRequest.AUTO;
        }

        lastEnabled = DriverStation.isEnabled();

        switch (brakeModeRequested) {
            case ALWAYS_BRAKE -> {
                setBrakeMode(true);
            }
            case ALWAYS_COAST -> {
                setBrakeMode(false);
            }
            case AUTO -> {
                if (DriverStation.isEnabled()) {
                    setBrakeMode(false);
                } else if (lastMovementTimer.hasElapsed(kCoastWaitTimeSeconds.get())) {
                    setBrakeMode(true);
                }
            }
        }

        switch (currentMode) {
            case TELEOP -> {
                desiredSpeeds = teleopController.update();
                brakeModeRequested =
                        teleopController.shouldBrake() ? BrakeModeRequest.ALWAYS_BRAKE : BrakeModeRequest.AUTO;
                if (headingController != null) {
                    desiredSpeeds.omegaRadiansPerSecond = headingController.update();
                }
            }
            case TRAJECTORY -> {
                desiredSpeeds = trajectoryController.update();
                if (headingController != null) {
                    desiredSpeeds.omegaRadiansPerSecond = headingController.update();
                }
            }
            case CHARACTERIZATION -> {
                for (Module module : modules) {
                    module.runCharacterization(0.0, characterizationInput);
                }
            }
            case WHEEL_RADIUS_CHARACTERIZATION -> {
                desiredSpeeds = new ChassisSpeeds(0, 0, characterizationInput);
            }
            default -> {}
        }
    }

    private void optimizeSwerveModuleSetpoints(ModuleLimits currentModuleLimits) {
        if (currentMode != SwerveMode.CHARACTERIZATION && !modulesOrienting) {
            currentSetpoint = setpointGenerator.generateSetpoint(
                    currentModuleLimits, currentSetpoint, desiredSpeeds, GlobalConstants.kLooperDT);

            SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
            SwerveModuleState[] optimizedSetpointTorques = new SwerveModuleState[4];

            for (int i = 0; i < 4; i++) {

                optimizedSetpointStates[i] =
                        SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());
                optimizedSetpointTorques[i] = new SwerveModuleState(0.0, optimizedSetpointStates[i].angle);
                modules[i].runSetpoint(optimizedSetpointStates[i], optimizedSetpointTorques[i]);
            }

            Logger.recordOutput("Swerve/SwerveStates/Setpoints", optimizedSetpointStates);
            Logger.recordOutput("Swerve/SwerveStates/Torques", optimizedSetpointTorques);
        }
    }

    private void logRecordOutputsSwerve() {
        Logger.recordOutput("Swerve/SwerveMode", currentMode);
        Logger.recordOutput("Swerve/DesiredSpeeds", desiredSpeeds);
        Logger.recordOutput("Swerve/SetpointSpeeds", currentSetpoint.chassisSpeeds());
        Logger.recordOutput("Swerve/SwerveStates/DesiredBeforePoofs", kKinematics.toSwerveModuleStates(desiredSpeeds));
        // Logger.recordOutput("Swerve/SwerveStates/SetpointState",
        //         currentSetpoint.moduleStates());
    }

    public void acceptTeleopInput(
            double controllerX, double controllerY, double controllerOmega, boolean robotRelative, boolean slow) {
        if (DriverStation.isTeleopEnabled()) {
            teleopController.acceptDriveInput(controllerX, controllerY, controllerOmega, robotRelative, slow);
        }
    }

    public void setTrajectory(ChoreoTrajectory traj) {
        if (DriverStation.isAutonomousEnabled()) {
            currentMode = SwerveMode.TRAJECTORY;
            trajectoryController = new ChoreoTrajectoryController(traj);
        }
    }

    public void clearTrajectory() {
        currentMode = SwerveMode.TELEOP;
        trajectoryController = null;
        RobotState.getInstance().addTrajectoryVelocityData(new Twist2d());
    }

    @AutoLogOutput(key = "Swerve/TrajectoryCompleted")
    public boolean isTrajectoryCompleted() {
        return trajectoryController != null && trajectoryController.isFinished();
    }

    public void setHeadingGoal(Supplier<Rotation2d> headingSupplier) {
        headingController = new HeadingController(headingSupplier);
    }

    public void clearHeadingGoal() {
        headingController = null;
    }

    @AutoLogOutput(key = "Swerve/AtHeadingGoal")
    public boolean atHeadingGoal() {
        return headingController != null && headingController.atGoal();
    }

    /** Runs forwards at the commanded voltage or amps. */
    public void runCharacterization(double input) {
        currentMode = SwerveMode.CHARACTERIZATION;
        characterizationInput = input;
    }

    /** Disables the characterization mode. */
    public void endCharacterization() {
        currentMode = SwerveMode.TELEOP;
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (Module module : modules) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    /** Get the position of all drive wheels in radians. */
    public double[] getWheelRadiusCharacterizationPosition() {
        return Arrays.stream(modules).mapToDouble(Module::getPositionRads).toArray();
    }

    /** Runs in a circle at omega. */
    public void runWheelRadiusCharacterization(double omegaSpeed) {
        currentMode = SwerveMode.WHEEL_RADIUS_CHARACTERIZATION;
        characterizationInput = omegaSpeed;
    }

    public void setBrakeMode(boolean enable) {
        if (brakeModeEnabled != enable) {
            Arrays.stream(modules).forEach((module) -> module.setBrakeMode(enable));
        }
        brakeModeEnabled = enable;
    }

    public void setTeleopDriveController(TeleopDriveController controller) {
        teleopController = controller;
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "Swerve/SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
    }

    /**
     * Returns the measured speeds of the robot in the robot's frame of reference.
     */
    @AutoLogOutput(key = "Swerve/MeasuredSpeeds")
    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.kKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns command that orients all modules to {@code orientation}, ending when
     * the modules have
     * rotated.
     */
    public Command orientModules(Rotation2d orientation) {
        return orientModules(new Rotation2d[] {orientation, orientation, orientation, orientation});
    }

    /**
     * Returns command that orients all modules to {@code orientations[]}, ending
     * when the modules
     * have rotated.
     */
    public Command orientModules(Rotation2d[] orientations) {
        return run(() -> {
                    SwerveModuleState[] states = new SwerveModuleState[4];
                    for (int i = 0; i < orientations.length; i++) {
                        modules[i].runSetpoint(
                                new SwerveModuleState(0.0, orientations[i]),
                                new SwerveModuleState(0.0, new Rotation2d()));
                        states[i] = new SwerveModuleState(0.0, modules[i].getAngle());
                    }
                    currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), states);
                })
                .until(() -> {
                    double[] differences = Arrays.stream(modules)
                            .mapToDouble(module -> module.getAngle()
                                    .minus(module.getSetpointState().angle)
                                    .getDegrees())
                            .toArray();
                    Logger.recordOutput("Swerve/OrientErrors", differences);
                    return Arrays.stream(differences).allMatch(x -> Math.abs(x) < 2.0);
                    // Arrays.stream(mModules)
                    // .allMatch(
                    // module ->
                    // EqualsUtil.epsilonEquals(
                    // module.getAngle().getDegrees(),
                    // module.getSetpointState().angle.getDegrees(),
                    // 2.0)))
                })
                .beforeStarting(() -> modulesOrienting = true)
                .finallyDo(() -> modulesOrienting = false)
                .withName("Orient Modules");
    }

    public static Rotation2d[] getStraightConfiguration() {
        return IntStream.range(0, 4).mapToObj(Rotation2d::new).toArray(Rotation2d[]::new);
    }

    public static Rotation2d[] getXConfiguration() {
        return Arrays.stream(SwerveConstants.kModuleTranslations)
                .map(Translation2d::getAngle)
                .toArray(Rotation2d[]::new);
    }

    public static Rotation2d[] getCircleConfiguration() {
        final Rotation2d offset = new Rotation2d(Math.PI / 2.0);
        return Arrays.stream(SwerveConstants.kModuleTranslations)
                .map(translation -> translation.getAngle().plus(offset))
                .toArray(Rotation2d[]::new);
    }
}
