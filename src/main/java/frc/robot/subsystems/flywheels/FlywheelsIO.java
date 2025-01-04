// CopyBack (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
    // TODO delete uneeded parameters
    @AutoLog
    class FlywheelsIOInputs {
        public boolean frontConnected = true;
        public boolean backConnected = true;

        public double frontPositionRads = 0.0;
        public double frontVelocityRPS = 0.0;
        public double frontAppliedVolts = 0.0;
        public double frontSupplyCurrentAmps = 0.0;
        public double frontTorqueCurrentAmps = 0.0;
        public double frontTempCelsius = 0.0;

        public double backPositionRads = 0.0;
        public double backVelocityRPS = 0.0;
        public double backAppliedVolts = 0.0;
        public double backSupplyCurrentAmps = 0.0;
        public double backTorqueCurrentAmps = 0.0;
        public double backTempCelsius = 0.0;
    }

    /** Update inputs */
    default void updateInputs(FlywheelsIOInputs inputs) {}

    /** Run both motors at voltage */
    default void runVolts(double motorVolts) {}

    /** Stop both motors */
    default void stop() {}

    /** Run both flywheels at velocity in rpm */
    default void runVelocity(double motorRpm, double motorFeedforward) {}

    /** Config PID values for both motors */
    default void setPID(double kP, double kI, double kD) {}

    /** Config FF values for both motors */
    default void setFF(double kS, double kV, double kA) {}

    /** Run flywheels at voltage */
    default void runCharacterization(double input) {}
}
