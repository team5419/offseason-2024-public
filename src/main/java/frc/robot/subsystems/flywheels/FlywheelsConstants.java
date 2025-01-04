package frc.robot.subsystems.flywheels;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class FlywheelsConstants {

    public static final FlywheelsConfig kFlywheelsConfig =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> new FlywheelsConfig(Ports.kFlywheelsOneID, Ports.kFlywheelsTwoID);
                case SIMBOT -> new FlywheelsConfig(0, 0);
            };

    public static final double kFlywheelGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> 5 / 3;
                case SIMBOT -> 1.0;
            };

    public static final Gains gains =
            switch (GlobalConstants.getRobotType()) {
                case ORBIT -> new Gains(0.75, 0, 0.0, 0.3, 0.114, 0.0);
                    // ! Make sure velocity above 30! Spin up time around 4.8 sec 0 to 80 rps
                case SIMBOT -> new Gains(0.01, 0.008, 0.00015, 0.01, 0.00103, 0.0);
            };

    public static final double flywheelSimJKG = 0.00363458292;
    public static final double kFlywheelFrequency = 100.0;
    public static final double kSupplyCurrentLimit = 40.0;
    public static final double kFlywheelsTolerance = 7; // in RPS

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public record FlywheelsConfig(int frontID, int backID) {}
}
