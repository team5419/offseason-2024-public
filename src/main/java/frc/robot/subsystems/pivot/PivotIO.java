package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
        public boolean rightMotorConnected = true;
        public boolean leftMotorConnected = true;

        public Rotation2d rightPosition = new Rotation2d();
        public Rotation2d leftPosition = new Rotation2d();
        public double velocityRadsPerSecond = 0.0;

        public double[] appliedVolts = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    default void updateInputs(PivotIOInputs inputs) {}

    /** Sets the arm config, must be called before other methods. */
    default void runSetpoint(double goal, double feedforward) {}

    default void runVolts(double volts) {}

    default void runCurrent(double amps) {}

    default void stop() {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double kP, double kI, double kD) {}
}
