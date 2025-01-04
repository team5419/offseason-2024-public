package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.statemachines.IntakeStateMachine;
import frc.robot.subsystems.statemachines.IntakeStateMachine.IntakeState;
import frc.robot.subsystems.statemachines.Superstructure;
import frc.robot.subsystems.statemachines.Superstructure.SuperState;

public class WaitForState extends Command {

    private final Superstructure ss;
    private final IntakeStateMachine ism;
    private Runnable request;
    private SuperState goalState;
    private boolean overrideExit = false;

    public WaitForState(
            Runnable request,
            SuperState goalState,
            Superstructure superstructure,
            IntakeStateMachine intakeStateMachine) {
        this.request = request;
        this.goalState = goalState;
        ss = superstructure;
        ism = intakeStateMachine;
    }

    public WaitForState(SuperState goalState, Superstructure superstructure, IntakeStateMachine intakeStateMachine) {
        this.request = () -> {};
        this.goalState = goalState;
        ss = superstructure;
        ism = intakeStateMachine;
        overrideExit = true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        request.run();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ss.getCurrentState() == goalState || (ism.getCurrentState() == IntakeState.INTAKE && !overrideExit);
    }
}
