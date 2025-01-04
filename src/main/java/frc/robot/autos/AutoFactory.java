package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.WaitForState;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.statemachines.IntakeStateMachine;
import frc.robot.subsystems.statemachines.Superstructure;
import frc.robot.subsystems.statemachines.Superstructure.SuperState;
import frc.robot.subsystems.swerve.Swerve;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
public class AutoFactory {

    private final Alliance alliance;

    private final RobotContainer robotContainer;
    private final RobotState robotState;
    private final Swerve swerve;
    private final Superstructure superstructure;
    private final IntakeStateMachine intakeStateMachine;
    private boolean trajectoriesLoaded = false;

    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    public AutoFactory(final Alliance alliance, final RobotContainer robotContainer) {
        this.alliance = alliance;
        this.robotContainer = robotContainer;
        swerve = this.robotContainer.getSwerve();
        superstructure = robotContainer.getSuperstructure();
        intakeStateMachine = robotContainer.getIntakeStateMachine();
        robotState = RobotState.getInstance();
    }

    /** Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */
    public Command createIdleCommand() {
        return Commands.idle();
    }

    public Command createSpkcSeven() {
        return new SequentialCommandGroup(
                resetOdometry(Location.SPKC, Location.SEVEN),
                requestShoot(),
                followTrajectory(Location.SPKC, Location.SEVEN));
    }

    public Command createZeroEight() {
        return new SequentialCommandGroup(
                resetOdometry(Location.SPKC, Location.EIGHT),
                requestShoot(),
                followTrajectory(Location.SPKC, Location.EIGHT),
                requestIntake(),
                followTrajectory(Location.EIGHT, Location.SPKC),
                requestShoot());
    }

    public Command createZeroEightSevenSix() {
        return new SequentialCommandGroup(
                resetOdometry(Location.SPKC, Location.EIGHT),
                requestShoot(),
                followTrajectory(Location.SPKC, Location.EIGHT),
                requestIntake(),
                followTrajectory(Location.EIGHT, Location.SPKC),
                requestShoot(),
                followTrajectory(Location.SPKC, Location.SEVEN),
                requestIntake(),
                followTrajectory(Location.SEVEN, Location.SPKC),
                requestShoot(),
                followTrajectory(Location.SPKC, Location.SIX),
                requestIntake(),
                followTrajectory(Location.SIX, Location.SPKC),
                requestShoot());
    }

    private Command resetOdometry(Location start, Location end) {
        // return Commands.runOnce(() -> robotState.resetPose(
        //     AllianceFlipUtil.apply(loadChorSegment(start, end).getInitialPose())));
        var initial = loadChorSegment(start, end).getInitialPose();
        return Commands.runOnce(() -> robotState.resetPose(AllianceFlipUtil.apply(
                new Pose2d(initial.getTranslation(), initial.getRotation().plus(Rotation2d.fromDegrees(180))))));
    }

    private Command followTrajectory(Location start, Location end) {
        return new InstantCommand(() -> robotContainer.getSwerve().setTrajectory(loadChorSegment(start, end)))
                .andThen(clearTrajectory());
    }

    private Command clearTrajectory() {
        return new WaitUntilCommand(() -> swerve.isTrajectoryCompleted())
                .andThen(new InstantCommand(() -> robotContainer.getSwerve().clearTrajectory()));
    }

    private Command requestShoot() {
        return new WaitForState(superstructure::requestShoot, SuperState.STOW, superstructure, intakeStateMachine);
    }

    private Command requestIntake() {
        return new WaitForState(superstructure::requestIntake, SuperState.INTAKE, superstructure, intakeStateMachine);
    }

    public void preloadTrajectoryClass(AutoSegment segment) {
        // This is done because Java loads classes lazily. Calling this here loads the trajectory class which
        // is used to follow paths and saves user code ms loop time at the start of auto.
        if (!trajectoriesLoaded) {
            trajectoriesLoaded = true;
            @SuppressWarnings("unused") // Keep this pls
            var trajectory = new PathPlannerTrajectory(
                    segment.path(),
                    RobotState.getInstance().getChassisSpeeds(),
                    RobotState.getInstance().getOdometryPose().getRotation());
        }
    }

    /**
     * load segments of auto paths
     */
    public AutoSegment loadSegment(final Location start, final Location end) {
        var name = "%S_TO_%S".formatted(start, end);
        var path = PathPlannerPath.fromPathFile("%S_%S".formatted(alliance, name));

        return new AutoSegment(start, end, name, path);
    }

    public ChoreoTrajectory loadChorSegment(final Location start, final Location end) {
        // var name = "%S_TO_%S".formatted(start, end);
        // return Choreo.getTrajectory("%S_%S".formatted(alliance, name));
        return Choreo.getTrajectory("%S_%S".formatted(start.getName(), end.getName()));
    }
}
