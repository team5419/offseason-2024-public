package frc.robot.subsystems.swerve.controllers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface TrajectoryController {

    public ChassisSpeeds update();

    public boolean isFinished();
}
