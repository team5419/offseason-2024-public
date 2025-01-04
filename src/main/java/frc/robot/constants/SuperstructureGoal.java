package frc.robot.constants;

import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.pivot.Pivot;

public record SuperstructureGoal(double pivotAngle, double flywheelsSpeed) {

    public SuperstructureGoal(Pivot.PivotGoal pivotAngle, Flywheels.FlywheelsGoal flywheelsSpeed) {
        this(
                pivotAngle.getPivotAngle().getAsDouble(),
                flywheelsSpeed.getFlywheelsSpeed().getAsDouble());
    }

    public static final SuperstructureGoal STOW = new SuperstructureGoal(0, 0); // TODO: get actual values
    // TODO: define more states here

    public SuperstructureGoal withPivotAngle(double newPivotAngle) {
        return new SuperstructureGoal(newPivotAngle, flywheelsSpeed);
    }

    public SuperstructureGoal withFlywheelsSpeed(double newFlywheelsSpeed) {
        return new SuperstructureGoal(pivotAngle, newFlywheelsSpeed);
    }
}
