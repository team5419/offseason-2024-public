package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import frc.robot.lib.swerve.ModuleLimits;
import lombok.Builder;

public class SwerveConstants {

    public static final Matrix<N3, N1> kOdometryStateStdDevs =
            switch (GlobalConstants.getRobotType()) {
                default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
            };

    public static final DriveConfig kDriveConfig =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> DriveConfig.builder()
                        .wheelRadius(Units.inchesToMeters(2.00)) // TODO: tune this
                        .trackWidthX(Units.inchesToMeters(20.415))
                        .trackWidthY(Units.inchesToMeters(20.415))
                        .bumperWidthX(Units.inchesToMeters(34.0))
                        .bumperWidthY(Units.inchesToMeters(34.0))
                        .maxLinearVelocity(Units.feetToMeters(16.5))
                        .maxLinearAcceleration(Units.feetToMeters(52.0))
                        .maxAngularVelocity(14.0)
                        .maxAngularAcceleration(9.0)
                        .build();
                default -> DriveConfig.builder()
                        .wheelRadius(Units.inchesToMeters(2.0))
                        .trackWidthX(Units.inchesToMeters(28.5))
                        .trackWidthY(Units.inchesToMeters(28.5))
                        .bumperWidthX(Units.inchesToMeters(34.0))
                        .bumperWidthY(Units.inchesToMeters(34.0))
                        .maxLinearVelocity(Units.feetToMeters(16.5))
                        .maxLinearAcceleration(Units.feetToMeters(50.0))
                        .maxAngularVelocity(12.0)
                        .maxAngularAcceleration(6.0)
                        .build();
            };

    public static final ModuleConfig[] kModuleConfig =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> new ModuleConfig[] {
                    new ModuleConfig(
                            Ports.kFrontLeftDriveID,
                            Ports.kFrontLeftSteerID,
                            Ports.kFrontLeftEncoderID,
                            Rotation2d.fromRotations(0.06689453125),
                            false,
                            true), // fl
                    new ModuleConfig(
                            Ports.kFrontRightDriveID,
                            Ports.kFrontRightSteerID,
                            Ports.kFrontRightEncoderID,
                            Rotation2d.fromRotations(-0.00341796875),
                            false,
                            false), // fr
                    new ModuleConfig(
                            Ports.kBackLeftDriveID,
                            Ports.kBackLeftSteerID,
                            Ports.kBackLeftEncoderID,
                            Rotation2d.fromRotations(-0.4814453125),
                            false,
                            true), // bl
                    new ModuleConfig(
                            Ports.kBackRightDriveID,
                            Ports.kBackRightSteerID,
                            Ports.kBackRightEncoderID,
                            Rotation2d.fromRotations(-0.1865234375),
                            false,
                            false) // br
                };
                case SIMBOT -> {
                    ModuleConfig[] configs = new ModuleConfig[4];
                    for (int i = 0; i < configs.length; i++)
                        configs[i] = new ModuleConfig(0, 0, 0, new Rotation2d(0), false, false);
                    yield configs;
                }
            };

    public static final ModuleConstants kModuleConstants =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> ModuleConstants.builder()
                        .ffkS(0.2)
                        .ffkV(0.11) // this is the theoretical value
                        .ffkT(1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp)
                        .drivekP(0.0)
                        .drivekD(0.0)
                        .turnkP(150.0)
                        .turnkD(2.0)
                        .driveReduction(Mk4NReductions.L3_PLUS.reduction)
                        .turnReduction(Mk4NReductions.TURN.reduction)
                        .build();
                case SIMBOT -> new ModuleConstants(
                        0.0074,
                        0.12046,
                        0.0,
                        0.1,
                        0.0,
                        10.0,
                        0.0,
                        Mk4NReductions.L3_PLUS.reduction,
                        Mk4NReductions.TURN.reduction);
                default -> new ModuleConstants(
                        0, 0, 0, 0, 0, 0, 0, Mk4NReductions.L3_PLUS.reduction, Mk4NReductions.TURN.reduction);
            };

    public static final String[] kModuleNames = new String[] {"FL", "FR", "BL", "BR"};

    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kDriveConfig.trackWidthX() / 2.0, kDriveConfig.trackWidthY() / 2.0),
        new Translation2d(kDriveConfig.trackWidthX() / 2.0, -kDriveConfig.trackWidthY() / 2.0),
        new Translation2d(-kDriveConfig.trackWidthX() / 2.0, kDriveConfig.trackWidthY() / 2.0),
        new Translation2d(-kDriveConfig.trackWidthX() / 2.0, -kDriveConfig.trackWidthY() / 2.0),
    };

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);

    public static final double kOdometryFreq =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> 250.0;
                case SIMBOT -> 25.0;
            };

    public static final ModuleLimits kModuleLimits = new ModuleLimits(
            kDriveConfig.maxLinearVelocity(), kDriveConfig.maxLinearAcceleration(), Units.degreesToRadians(1080.0));

    public static final TrajectoryConstants kTrajectoryConstants =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> TrajectoryConstants.builder()
                        .linearKp(3.0)
                        .linearKd(0.0)
                        .thetaKp(1.0)
                        .thetaKd(0.0)
                        .build();
                case SIMBOT -> TrajectoryConstants.builder()
                        .linearKp(4.0)
                        .thetaKp(4.0)
                        .build();
            };

    public static final HeadingControllerConstants kHeadingControllerConstants =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> HeadingControllerConstants.builder().kP(3.0).build();
                case SIMBOT -> HeadingControllerConstants.builder().kP(10.0).build();
            };

    public record ModuleConfig(
            int driveID,
            int turnID,
            int cancoderID,
            Rotation2d absoluteEncoderOffset,
            boolean turnMotorInverted,
            boolean driveMotorInverted) {}

    @Builder
    public record DriveConfig(
            double wheelRadius,
            double trackWidthX,
            double trackWidthY,
            double bumperWidthX,
            double bumperWidthY,
            double maxLinearVelocity,
            double maxLinearAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration) {

        public double driveBaseRadius() {
            return Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
        }
    }

    @Builder
    public record ModuleConstants(
            double ffkS,
            double ffkV,
            double ffkT,
            double drivekP,
            double drivekD,
            double turnkP,
            double turnkD,
            double driveReduction,
            double turnReduction) {}

    @Builder
    public record TrajectoryConstants(double linearKp, double linearKd, double thetaKp, double thetaKd) {}

    @Builder
    public record HeadingControllerConstants(double kP, double kD) {}

    private enum Mk4iReductions {
        L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
        L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
        TURN((150.0 / 7.0));

        final double reduction;

        private Mk4iReductions(double reduction) {
            this.reduction = reduction;
        }
    }

    private enum Mk4NReductions {
        L2_PLUS((50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0)),
        L3_PLUS((50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)),
        TURN(18.75 / 1);

        final double reduction;

        private Mk4NReductions(double reduction) {
            this.reduction = reduction;
        }
    }
}
