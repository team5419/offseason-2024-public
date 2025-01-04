package frc.robot.autos;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * A {@link edu.wpi.first.wpilibj.smartdashboard.SendableChooser} for selecting an autonomous program.
 * <br/><br/>
 * <p>
 * How to add a new autonomous program.
 *     <ol>
 *         <li>
 *             Add a new value to {@link Auto}<br/>
 *             The name of the value should use screaming snake case
 *             </li>
 *             <li>
 *                 Add a new method in {@link AutoFactory} that returns a {@link edu.wpi.first.wpilibj2.command.Command}.
 *             </li>
 *             <li>
 *                 Add a new {@link AutoProgram} to {@link AutoChooser#autoProgramsList}.
 *             </li>
 *             <li>
 *                 Implement the autonomous program factory method.
 *             </li>
 *             <li>
 *                 Test, test, and test some more.
 *             </li>
 *         </ol>
 * </p>
 */
public class AutoChooser extends SendableChooser<Auto> {
    private final Map<Auto, AutoProgram> programs;

    private final Map<Alliance, Map<Auto, Command>> commandCache;

    private final Map<Alliance, AutoFactory> autoFactories;

    private AutoChooser(final Map<Auto, AutoProgram> programs, final Map<Alliance, AutoFactory> autoFactories) {
        this.programs = programs;
        this.autoFactories = autoFactories;
        commandCache = Stream.of(DriverStation.Alliance.values())
                .map(alliance -> Map.entry(alliance, new HashMap<Auto, Command>()))
                .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
    }

    private static final List<AutoProgram> autoProgramsList = List.of(
            new AutoProgram(Auto.IDLE, "IDLE", AutoFactory::createIdleCommand),
            new AutoProgram(Auto.ZERO_SEVEN, "07", AutoFactory::createSpkcSeven),
            new AutoProgram(Auto.ZERO_EIGHT, "08", AutoFactory::createZeroEight),
            new AutoProgram(Auto.ZERO_EIGHT_SEVEN_SIX, "0876", AutoFactory::createZeroEightSevenSix));

    /**
     * Create a new <code>AutoChooser</code>
     *
     * @param robotContainer A {@link RobotContainer}
     * @return A new <code>AutoChooser</code> populated with the programs defined in the private field {@link AutoChooser#autoProgramsList}.
     */
    public static AutoChooser create(final RobotContainer robotContainer) {
        var autoFactories = Stream.of(DriverStation.Alliance.values())
                .map(alliance -> Map.entry(alliance, new AutoFactory(alliance, robotContainer)))
                .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
        var programs = autoProgramsList.stream()
                .map(program -> Map.entry(program.getAuto(), program))
                .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

        var autoChooser = new AutoChooser(programs, autoFactories);

        autoProgramsList.forEach(program -> {
            if (program.getAuto() == Auto.IDLE) {
                autoChooser.setDefaultOption(program.getLabel(), program.getAuto());
            } else {
                autoChooser.addOption(program.getLabel(), program.getAuto());
            }
        });

        autoChooser.reset(null);

        Shuffleboard.getTab("Auto Chooser").add("Auto Chooser", autoChooser);

        Shuffleboard.getTab("Auto Chooser")
                .addString("Selected Auto", () -> autoChooser.getSelected().name())
                .withPosition(12, 3)
                .withSize(6, 2)
                .withWidget(BuiltInWidgets.kTextView);

        return autoChooser;
    }

    /**
     * Update the <code>AutoChooser</code><br/><br/>
     *
     * <p>
     * The commands for the selected autonomous program are loaded and cached (if not already present)
     * and the selected program is sent to the {@link edu.wpi.first.wpilibj.shuffleboard.Shuffleboard} under the key <code>Auto/Selected</code>.
     * </p>
     */
    public void update() {
        var selected = getSelected();

        Stream.of(DriverStation.Alliance.values()).forEach(alliance -> {
            commandCache.get(alliance).computeIfAbsent(selected, auto -> loadCommand(alliance, auto));
        });
    }

    /**
     * Reset the caches behind this <code>AutoChooser</code>
     *
     * @param key Optional {@link edu.wpi.first.networktables.NetworkTable} key.
     *            If provided the selected value of the entry at this location will also be reset.
     */
    public void reset(final String key) {
        Stream.of(DriverStation.Alliance.values())
                .forEach(alliance -> commandCache.get(alliance).clear());

        if (key != null) {
            var table = NetworkTableInstance.getDefault().getTable(key);
            table.putValue("selected", NetworkTableValue.makeString("%s".formatted(Auto.IDLE)));
        }
    }

    /**
     * Get the {@link Command} for the selected autonomous program, if available.
     *
     * @return The {@link Command} for the selected autonomous program as an {@link Optional} if available,
     * otherwise {@link Optional#empty}.
     */
    public Optional<Command> getSelectedCommand() {
        var selected = getSelected();
        return DriverStation.getAlliance().map(alliance -> {
            System.out.printf("Running program %s/%s\n", alliance, selected);
            System.out.println(commandCache.get(alliance));
            System.out.println(commandCache.get(alliance).get(selected));
            return commandCache.get(alliance).get(selected);
        });
    }

    public AutoProgram getProgram() {
        return programs.get(getSelected());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.publishConstString("selected", "%s".formatted(Auto.IDLE));
    }

    private Command loadCommand(final DriverStation.Alliance alliance, final Auto auto) {
        var program = programs.get(auto);

        System.out.printf("Loading command %s/%s\n", alliance, auto);

        return program.getCommand(autoFactories.get(alliance));
    }
}
