package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class PivotConstants {
    // Prob should do switch case for sim stuff
    public static final Gains gains =
            switch (GlobalConstants.getRobotType()) {
                    // todo change these values eventually
                case ORBIT -> new Gains(175.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                case SIMBOT -> new Gains(70.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
            };
    public static final PivotConfig kPivotConfig =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> new PivotConfig(Ports.kPivotOneID, Ports.kPivotTwoID);
                case SIMBOT -> new PivotConfig(0, 0);
            };

    // Motion magic configs
    // TODO increase both of these
    public static final double kMotionMagicCruiseVel = 240;
    public static final double kMotionMagicAccel = 300;

    // not using for now
    // public static final double kMotionMagicJerk = 480;

    public static final double kGearRatio = 310 / 3;
    public static final double kPositionToleranceDegrees = 2;
    public static final double kPivotFrequency = 100.0;
    public static final double kStatorCurrentLimit = 40;

    // sim constants
    public static final double kMinAngle = 0;
    public static final double kMaxAngle = 90;
    public static final double kLengthMeters = .3;
    public static final double kMassKg = 10;
    public static final double kReduction = 50.0;
    public static final double kStartingAngle = 26;
    public static final double kEstMoi = SingleJointedArmSim.estimateMOI(kLengthMeters, kMassKg);

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public record MotionMagicConfigs(
            double kMotionMagicCruiseVel, double kMotionMagicAcceleration, double kMotionMagicJerk) {}

    public record PivotConfig(int leftID, int rightID) {}
}
