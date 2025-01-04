// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {
        public boolean leftMotorConnected = true;
        public boolean rightMotorConnected = true;

        public double leftMotorPositionRads = 0.0;
        public double leftMotorVelocityRPS = 0.0;
        public double leftMotorAppliedVolts = 0.0;
        public double leftMotorSupplyCurrentAmps = 0.0;
        public double leftMotorTorqueCurrentAmps = 0.0;
        public double leftMotorTempCelsius = 0.0;

        public double rightMotorPositionRads = 0.0;
        public double rightMotorVelocityRPS = 0.0;
        public double rightMotorAppliedVolts = 0.0;
        public double rightMotorSupplyCurrentAmps = 0.0;
        public double rightMotorTorqueCurrentAmps = 0.0;
        public double rightMotorTempCelsius = 0.0;
    }

    /** Update inputs */
    default void updateInputs(IntakeIOInputs inputs) {}

    // TODO: get rid of redundant parameters
    // or at least helper methods with only one
    // instead of parameters for both left and right motors

    /** Run both motors at voltage */
    default void runVolts(double motorVolts) {}

    /** Stop both motors */
    default void stop() {}

    /** Run left and right motors at velocity in rpm */
    default void runVelocity(double motorRPS, double ff) {}

    /** Config PID values for both motors */
    default void setPID(double kP, double kI, double kD) {}

    /** Run left motor at voltage */
    default void runCharacterizationLeftMotor(double input) {}

    /** Run right motor at voltage */
    default void runCharacterizationRightMotor(double input) {}
}
