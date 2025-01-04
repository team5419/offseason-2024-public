package frc.robot.subsystems.intake;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakeConstants {

    public static final IntakeConfig kIntakeConfig =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> new IntakeConfig(Ports.kIntakeIndexerLeftID, Ports.kIntakeIndexerRightID);
                case SIMBOT -> new IntakeConfig(0, 0);
            };

    public static final double kIntakeGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> 1.0;
                case SIMBOT -> 1.0;
            };

    public static final Gains gains =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> new Gains(0.5, 0.0, 0.0, 0.33329, 0.12, 0.0);
                case SIMBOT -> new Gains(0.010, 0.008, 0.00015, 0.01, 0.00103, 0.0);
            };

    public static final double kSupplyCurrentLimit = 60.0;
    public static final double kIntakeFrequency = 200.0;
    public static final double kVelocityTolerance = 10.0;

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public record IntakeConfig(int leftID, int rightID) {}
}
