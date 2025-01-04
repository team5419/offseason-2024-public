// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoChooser;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.AprilTagLayoutType;
import frc.robot.constants.GlobalConstants;
import frc.robot.lib.RumbleThread;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhoton;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOSim;
import frc.robot.subsystems.detectionvision.DetectionVision;
import frc.robot.subsystems.detectionvision.DetectionVisionIOPhoton;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.statemachines.IntakeStateMachine;
import frc.robot.subsystems.statemachines.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    private CommandXboxController driver = new CommandXboxController(0);

    private CommandXboxController codriver = new CommandXboxController(1);

    @Getter
    private final AprilTagVision aprilTagVision;

    @Getter
    private final DetectionVision detectionVision;

    @Getter
    private final Flywheels flywheels;

    @Getter
    private final Intake intake;

    @Getter
    private final Leds leds = Leds.getInstance();

    @Getter
    private final Pivot pivot;

    @Getter
    private final IntakeStateMachine intakeStateMachine;

    @Getter
    private final Superstructure superstructure;

    @Getter
    private final Swerve swerve;

    private final LoggedDashboardChooser<AprilTagLayoutType> layoutChooser;
    private final Supplier<AprilTagLayoutType> layoutSupplier;

    @Getter
    private final AutoChooser autoChooser;

    public RobotContainer() {

        // Get driver station to stop
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure the layout chooser
        layoutChooser = new LoggedDashboardChooser<>("AprilTagLayout");
        layoutChooser.addDefaultOption("Official", FieldConstants.AprilTagLayoutType.OFFICIAL);
        layoutChooser.addOption("Speakers Only", FieldConstants.AprilTagLayoutType.SPEAKERS_ONLY);
        layoutChooser.addOption("Amps Only", FieldConstants.AprilTagLayoutType.AMPS_ONLY);
        layoutChooser.addOption("None", FieldConstants.AprilTagLayoutType.NONE);
        layoutSupplier = () -> {
            FieldConstants.AprilTagLayoutType type = layoutChooser.get();
            if (type == null) return FieldConstants.AprilTagLayoutType.OFFICIAL;
            return type;
        };

        // Set a custom brownout voltage (from Zach)
        // RobotController.setBrownoutVoltage(6.1);
        // Comment out when we are testing, so we don't mess up the batteries

        AprilTagVision tempAprilTagVision = null;
        DetectionVision tempDetectionVision = null;
        Flywheels tempFlywheels = null;
        Intake tempIntake = null;
        Pivot tempPivot = null;
        Swerve tempSwerve = null;

        if (GlobalConstants.getMode() != GlobalConstants.Mode.REPLAY) {
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> {
                    tempSwerve = new Swerve(
                            new GyroIOPigeon2(),
                            new ModuleIOTalonFX(SwerveConstants.kModuleConfig[0]),
                            new ModuleIOTalonFX(SwerveConstants.kModuleConfig[1]),
                            new ModuleIOTalonFX(SwerveConstants.kModuleConfig[2]),
                            new ModuleIOTalonFX(SwerveConstants.kModuleConfig[3]));

                    tempAprilTagVision = new AprilTagVision(
                            layoutSupplier,
                            new AprilTagVisionIOPhoton(layoutSupplier, 0),
                            new AprilTagVisionIOPhoton(layoutSupplier, 1));

                    tempDetectionVision = new DetectionVision(new DetectionVisionIOPhoton(0));

                    tempPivot = new Pivot(new PivotIOTalonFX());

                    tempFlywheels = new Flywheels(new FlywheelsIOTalonFX());

                    tempIntake = new Intake(new IntakeIOTalonFX());
                }
                case SIMBOT -> {
                    tempSwerve = new Swerve(
                            new GyroIO() {},
                            new ModuleIOSim(SwerveConstants.kModuleConfig[0]),
                            new ModuleIOSim(SwerveConstants.kModuleConfig[1]),
                            new ModuleIOSim(SwerveConstants.kModuleConfig[2]),
                            new ModuleIOSim(SwerveConstants.kModuleConfig[3]));

                    tempAprilTagVision = new AprilTagVision(
                            layoutSupplier,
                            new AprilTagVisionIOSim(layoutSupplier, 0),
                            new AprilTagVisionIOSim(layoutSupplier, 1));

                    tempPivot = new Pivot(new PivotIOSim());
                    tempIntake = new Intake(new IntakeIOSim());
                    tempFlywheels = new Flywheels(new FlywheelsIOSim());
                }
            }
        }

        if (tempSwerve == null)
            tempSwerve = new Swerve(
                    new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});

        if (tempAprilTagVision == null)
            tempAprilTagVision = new AprilTagVision(layoutSupplier, new AprilTagVisionIOPhoton(layoutSupplier, 0));

        if (tempDetectionVision == null) tempDetectionVision = new DetectionVision(new DetectionVisionIOPhoton(0) {});

        if (tempPivot == null) tempPivot = new Pivot(new PivotIO() {});

        if (tempFlywheels == null) tempFlywheels = new Flywheels(new FlywheelsIO() {});

        if (tempIntake == null) tempIntake = new Intake(new IntakeIO() {});

        aprilTagVision = tempAprilTagVision;
        detectionVision = tempDetectionVision;
        flywheels = tempFlywheels;
        intake = tempIntake;
        pivot = tempPivot;
        swerve = tempSwerve;

        intakeStateMachine = new IntakeStateMachine(intake, flywheels, pivot, swerve);
        superstructure = new Superstructure(this);

        autoChooser = AutoChooser.create(this);

        configureDefaultCommands();
        configureDriverBindings();
        // configureCodriverBindings();

        RumbleThread.getInstance().bindControllers(driver, codriver);

        // ? Uncomment the following if you want to test stuff ?//
        configureDevBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(swerve.run(() -> {
                    swerve.acceptTeleopInput(
                            -driver.getLeftY(),
                            -driver.getLeftX(),
                            -driver.getRightX(),
                            false,
                            driver.leftBumper().getAsBoolean());
                })
                .withName("Teleop Drive Input"));
    }

    private void configureDriverBindings() {

        driver.start(); // ! Unbound
        driver.back(); // ! Unbound

        driver.a().onTrue(Commands.runOnce(superstructure::requestStow));
        driver.b(); // ! Unbound
        // driver.b().onTrue(Commands.runOnce(superstructure::requestInterpolate));
        driver.x(); // ! Unbound
        driver.y()
                .onTrue(new InstantCommand(() -> RobotState.getInstance().resetGyro())
                        .ignoringDisable(true)); // reset gyro angle

        driver.povUp(); // ! Unbound
        driver.povDown(); // ! Unbound
        // driver.povDown().onTrue(Commands.runOnce(() -> pivot.setGoal(PivotGoal.STOW)));
        driver.povLeft(); // ! Unbound
        driver.povRight(); // ! Unbound

        driver.leftBumper(); // ! SWERVE SLOWMODE
        driver.rightBumper(); // ! Unbound

        driver.leftTrigger(0.1).onTrue(Commands.runOnce(superstructure::requestIntake));
        driver.rightTrigger(0.1).onTrue(Commands.runOnce(superstructure::requestShoot));
    }

    private void configureCodriverBindings() {

        codriver.start(); // ! Unbound
        codriver.back(); // ! Unbound

        codriver.a().onTrue(Commands.runOnce(superstructure::requestIdle));
        codriver.b().whileTrue(Commands.runOnce(() -> pivot.setGoal(PivotGoal.AMP)));
        codriver.b().onFalse(Commands.runOnce(() -> pivot.setGoal(PivotGoal.STOW)));
        codriver.x(); // ! Unbound
        codriver.y(); // ! Unbound

        codriver.povUp(); // ! Unbound
        codriver.povDown(); // ! Unbound
        codriver.povLeft(); // ! Unbound
        codriver.povRight(); // ! Unbound

        codriver.leftBumper(); // ! Unbound
        codriver.rightBumper(); // ! Unbound

        codriver.leftTrigger(0.1).onTrue(Commands.runOnce(() -> superstructure.setPivotShootGoal(PivotGoal.SUBWOOFER)));
        codriver.rightTrigger(0.1)
                .onTrue(Commands.runOnce(() -> superstructure.setPivotShootGoal(PivotGoal.STOCKPILE)));
    }

    /**
     * Use this function to test new features without
     * changing the current button bindings
     */
    private void configureDevBindings() {
        driver.start(); // ! Unbound
        driver.back(); // ! Unbound

        driver.a().onTrue(Commands.runOnce(superstructure::requestShoot));
        driver.b().onTrue(Commands.runOnce(superstructure::requestIntake));
        driver.x().onTrue(Commands.runOnce(superstructure::requestIdle));
        driver.y().onTrue(Commands.runOnce(superstructure::requestStow));

        driver.povUp().onTrue(Commands.runOnce(() -> pivot.setGoal(PivotGoal.AMP)));
        driver.povDown().onTrue(Commands.runOnce(() -> pivot.setGoal(PivotGoal.STOW)));
        driver.povLeft().whileTrue(Commands.run(() -> intake.runVelocity(-40)));
        driver.povLeft().onFalse(Commands.runOnce(intake::stop));
        driver.povRight().onTrue(Commands.runOnce(() -> pivot.setGoal(PivotGoal.SUBWOOFER)));

        driver.leftBumper(); // ! SWERVE SLOWMODE
        driver.rightBumper().whileTrue(Commands.run(() -> intake.runVelocity(-40)));
        driver.rightBumper().onFalse(Commands.runOnce(intake::stop));

        driver.leftTrigger(0.1).whileTrue(Commands.run(flywheels::runVelocity));
        driver.leftTrigger(0.1).onFalse(Commands.runOnce(flywheels::stop));

        driver.rightTrigger(0.1).whileTrue(Commands.run(() -> intake.runVelocity()));
        driver.rightTrigger(0.1).whileFalse(Commands.runOnce(intake::stop));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelectedCommand().orElse(Commands.idle());
    }
}
