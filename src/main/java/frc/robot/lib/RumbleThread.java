package frc.robot.lib;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleThread {

    private static RumbleThread instance = null;
    private Notifier notifier;
    private RumbleType type = RumbleType.kBothRumble;
    private CommandXboxController[] controllers;

    public enum ControllersToRumble {
        DRIVER,
        CODRIVER,
        ALL
    }

    // Get subsystem
    public static RumbleThread getInstance() {
        if (instance == null) instance = new RumbleThread();
        return instance;
    }

    private RumbleThread() {
        notifier = new Notifier(this::stopAllRumbles);
        notifier.stop();
    }

    public void bindControllers(CommandXboxController... controllers) {
        this.controllers = controllers;
    }

    private void stopAllRumbles() {
        if (controllers == null) return;
        for (CommandXboxController c : controllers) {
            c.getHID().setRumble(type, 0);
        }
    }

    public void setRumble(RumbleType type, double amount, double rumbleTime, ControllersToRumble controllersToRumble) {

        if (controllers == null) return;
        stopAllRumbles();

        switch (controllersToRumble) {
            case DRIVER:
                if (controllers.length > 0) controllers[0].getHID().setRumble(type, amount);
                break;

            case CODRIVER:
                if (controllers.length > 1 && controllers[1] != null)
                    controllers[1].getHID().setRumble(type, amount);
                break;

            default:
                for (CommandXboxController c : controllers) {
                    c.getHID().setRumble(type, amount);
                }
                break;
        }

        notifier.startSingle(rumbleTime);
    }
}
