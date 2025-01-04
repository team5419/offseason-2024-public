// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.module;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.Alert;
import frc.robot.lib.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Module {
    private static final LoggedTunableNumber drivekP =
            new LoggedTunableNumber("Swerve/Module/DrivekP", kModuleConstants.drivekP());
    private static final LoggedTunableNumber drivekD =
            new LoggedTunableNumber("Swerve/Module/DrivekD", kModuleConstants.drivekD());
    private static final LoggedTunableNumber drivekS =
            new LoggedTunableNumber("Swerve/Module/DrivekS", kModuleConstants.ffkS());
    private static final LoggedTunableNumber drivekV =
            new LoggedTunableNumber("Swerve/Module/DrivekV", kModuleConstants.ffkV());
    private static final LoggedTunableNumber turnkP =
            new LoggedTunableNumber("Swerve/Module/TurnkP", kModuleConstants.turnkP());
    private static final LoggedTunableNumber turnkD =
            new LoggedTunableNumber("Swerve/Module/TurnkD", kModuleConstants.turnkD());

    private final int index;
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged moduleInputs = new ModuleIOInputsAutoLogged();

    // private SimpleMotorFeedforward mFF =
    // new SimpleMotorFeedforward(kModuleConstants.ffkS(), kModuleConstants.ffkV(),
    // 0.0);
    @AutoLogOutput(key = "Swerve/Module{index}/SetpointState")
    @Getter
    private SwerveModuleState setpointState = new SwerveModuleState();

    // Alerts
    private final Alert driveMotorDisconnected;
    private final Alert turnMotorDisconnected;
    private final Alert encoderDisconnected;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveMotorDisconnected = new Alert(kModuleNames[index] + " drive motor disconnected!", Alert.AlertType.WARNING);
        turnMotorDisconnected = new Alert(kModuleNames[index] + " turn motor disconnected!", Alert.AlertType.WARNING);
        encoderDisconnected = new Alert(kModuleNames[index] + " encoder disconnected!", Alert.AlertType.WARNING);
    }

    /** Called while blocking odometry thread */
    public void updateInputs() {
        io.updateInputs(moduleInputs);
        Logger.processInputs("Swerve/Module" + index, moduleInputs);

        // Update ff and controllers
        // LoggedTunableNumber.ifChanged(
        // hashCode(),
        // () -> mFF = new SimpleMotorFeedforward(drivekS.get(), drivekV.get(), 0),
        // drivekS,
        // drivekV);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> io.setDrivePID(drivekP.get(), 0, drivekD.get(), drivekS.get(), drivekV.get()),
                drivekP,
                drivekD,
                drivekS,
                drivekV);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setTurnPID(turnkP.get(), 0, turnkD.get()), turnkP, turnkD);

        // Display alerts
        driveMotorDisconnected.set(!moduleInputs.driveMotorConnected);
        turnMotorDisconnected.set(!moduleInputs.turnMotorConnected);
        encoderDisconnected.set(!moduleInputs.encoderConnected);
    }

    /** Runs to {@link SwerveModuleState} */
    public void runSetpoint(SwerveModuleState setpoint, SwerveModuleState torqueFF) {
        setpointState = setpoint;
        // double wheelTorqueNm = torqueFF.speedMetersPerSecond;
        // double feedforward = (wheelTorqueNm / kModuleConstants.driveReduction()) *
        // kModuleConstants.ffkT();

        io.runDriveVelocitySetpoint(setpoint.speedMetersPerSecond / kDriveConfig.wheelRadius(), 0.0);
        io.runTurnPositionSetpoint(setpoint.angle.getRadians());
        // double wheelTorqueNm =
        // torqueFF.speedMetersPerSecond; // Using SwerveModuleState for torque for easy
        // logging
        // mIO.runDriveVelocitySetpoint(
        // setpoint.speedMetersPerSecond / kDriveConfig.wheelRadius(),
        // mFF.calculate(setpoint.speedMetersPerSecond / kDriveConfig.wheelRadius())
        // + ((wheelTorqueNm / kModuleConstants.driveReduction()) *
        // kModuleConstants.ffkT()));
    }

    /**
     * Runs characterization volts or amps depending on using voltage or current
     * control.
     */
    public void runCharacterization(double turnSetpointRads, double input) {
        io.runTurnPositionSetpoint(turnSetpointRads);
        io.runCharacterization(input);
    }

    /** Sets brake mode to {@code enabled}. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Stops motors. */
    public void stop() {
        io.stop();
    }

    /** Get all latest {@link SwerveModulePosition}'s from last cycle. */
    public SwerveModulePosition[] getModulePositions() {
        int minOdometryPositions =
                Math.min(moduleInputs.odometryDrivePositionsMeters.length, moduleInputs.odometryTurnPositions.length);
        SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
        for (int i = 0; i < minOdometryPositions; i++) {
            positions[i] = new SwerveModulePosition(
                    moduleInputs.odometryDrivePositionsMeters[i], moduleInputs.odometryTurnPositions[i]);
        }
        return positions;
    }

    /** Get turn angle of module as {@link Rotation2d}. */
    public Rotation2d getAngle() {
        return moduleInputs.turnAbsolutePosition;
    }

    /** Get position of wheel rotations in radians */
    public double getPositionRads() {
        return moduleInputs.drivePositionRads;
    }

    /** Get position of wheel in meters. */
    public double getPositionMeters() {
        return moduleInputs.drivePositionRads * kDriveConfig.wheelRadius();
    }

    /** Get velocity of wheel in m/s. */
    public double getVelocityMetersPerSec() {
        return moduleInputs.driveVelocityRadsPerSec * kDriveConfig.wheelRadius();
    }

    /** Get current {@link SwerveModulePosition} of module. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Get current {@link SwerveModuleState} of module. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Get velocity of drive wheel for characterization */
    public double getCharacterizationVelocity() {
        return moduleInputs.driveVelocityRadsPerSec;
    }
}
