package frc.robot.autos;

import lombok.Getter;

/**
 * Enum containing all locations for use in auto paths
 */
public enum Location {
    SPKC("SpkC"), // x: 1.35, y: 5.55, rot: 180 deg
    ONE("1"), // x: 8.3, y: 7.45, rot: -168 deg
    TWO("2"), // x: 8.3, y: 5.8, rot: 169 deg
    THREE("3"),
    FOUR("4"),
    FIVE("5"),
    SIX("6"), // x: 2.9, y: 7, rot: 0.88 rad
    SEVEN("7"), // x: 2.9, y: 5.55, rot: 180 deg
    EIGHT("8"); // x: 2.9, y: 4.1, rot: 150 deg

    @Getter
    private String name;

    private Location(String name) {
        this.name = name;
    }
}
