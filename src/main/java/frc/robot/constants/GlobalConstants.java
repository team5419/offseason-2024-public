package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.Alert;
import frc.robot.lib.Alert.AlertType;

/**
 * A class to hold the constants for the entire robot, as well as the mode of the robot (real or sim)
 */
public class GlobalConstants {

    // ! Use simbot for simulation, orbit for robot
    private static RobotType kRobotType = RobotType.ORBIT;

    public static final double kLooperDT = 0.02;
    public static final boolean kTuningMode = true;

    public static final double kCANErrorTimeThreshold = 0.5;
    public static final double kCANivoreErrorTimeThreshold = 0.5;
    public static final double kLowBatteryVoltage = 11.8;
    public static final double kLowBatteryDisabledTime = 1.5;
    public static final double kBrownoutVoltage = 6.0;

    public static final String kRIOName = "rio";
    public static final String kCANivoreName =
            switch (kRobotType) {
                default -> "Drivebase";
            };

    public static RobotType getRobotType() {
        if (RobotBase.isReal() && kRobotType == RobotType.SIMBOT) {
            new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR).set(true);
            kRobotType = RobotType.ORBIT;
        }
        return kRobotType;
    }

    public static Mode getMode() {
        return switch (kRobotType) {
            case ORBIT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT -> Mode.SIM;
        };
    }

    public enum RobotType {
        ORBIT,
        SIMBOT,
    }

    public enum Mode {
        REAL,
        REPLAY,
        SIM
    }

    public static void main(String... args) {
        if (kRobotType == RobotType.SIMBOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + kRobotType);
            System.exit(1);
        }
    }
}
