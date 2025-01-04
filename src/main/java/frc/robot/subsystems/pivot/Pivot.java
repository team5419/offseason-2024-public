package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private PivotVisualizer measured = new PivotVisualizer("Measured", Color.kAquamarine);
    private PivotVisualizer target = new PivotVisualizer("Target", Color.kGreen);

    private boolean isChacterization;
    private boolean brakeModeEnabled;

    private ArmFeedforward ff;

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", gains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", gains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", gains.kA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", gains.kG());

    private static final LoggedTunableNumber stowAngle = new LoggedTunableNumber("Pivot/Stow Angle", 1.0);
    private static final LoggedTunableNumber subwooferAngle = new LoggedTunableNumber("Pivot/Subwoofer Angle", 48);
    private static final LoggedTunableNumber ampAngle = new LoggedTunableNumber("Pivot/Amp Angle", 90);
    private static final LoggedTunableNumber stockpileAngle = new LoggedTunableNumber("Pivot/Stockpile Angle", 3);

    public static final LoggedTunableNumber voltage = new LoggedTunableNumber("Pivot/Voltage");

    public enum PivotGoal {
        STOW(stowAngle),
        LOWER(() -> 15), // for testing
        SUBWOOFER(subwooferAngle),
        AMP(ampAngle),
        STOCKPILE(stockpileAngle),
        INTERPOLATE(() -> RobotState.getInstance().getShotParameters().interpolatedArmAngle());

        @Getter
        private DoubleSupplier pivotAngle;

        private PivotGoal(DoubleSupplier pivotAngle) {
            this.pivotAngle = pivotAngle;
        }
    }

    @Getter
    @AutoLogOutput(key = "Pivot/Pivot Goal")
    private PivotGoal currentGoal = PivotGoal.STOW;

    public Pivot(PivotIO io) {
        this.io = io;
        io.setBrakeMode(false);
        ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot/inputs", inputs);
        Logger.recordOutput("Pivot/Goal angle", currentGoal.pivotAngle.getAsDouble());
        measured.update(inputs.rightPosition);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()), kS, kG, kV, kA);

        if (DriverStation.isDisabled()) {
            io.stop();
            io.setBrakeMode(false);
        } else if (!isChacterization && !brakeModeEnabled) {
            if (currentGoal == PivotGoal.STOW
                    && (inputs.leftPosition.getDegrees() < 5 && inputs.rightPosition.getDegrees() < 5)) {
                stop();
            } else {
                Logger.recordOutput("Pivot/is stopped", false);
                io.runSetpoint(
                        currentGoal.pivotAngle.getAsDouble(),
                        ff.calculate(inputs.rightPosition.getDegrees(), inputs.velocityRadsPerSecond));
            }
        }
    }

    // todo fix this (should call funcions on io not sim)
    // public void setGoal(PivotGoal desiredGoal) {
    //     currentGoal = desiredGoal;
    //     target.update(Rotation2d.fromDegrees(desiredGoal.pivotAngle.getAsDouble()));
    // }

    public void setGoal(PivotGoal desiredGoal, double ff) {
        currentGoal = desiredGoal;
        io.runSetpoint(desiredGoal.pivotAngle.getAsDouble(), ff);
    }

    public void setGoal(PivotGoal desiredGoal) {
        currentGoal = desiredGoal;
        io.runSetpoint(desiredGoal.pivotAngle.getAsDouble(), PivotConstants.gains.kV());
    }

    public void stop() {
        Logger.recordOutput("Pivot/is stopped", true);
        io.stop();
    }

    public double getAngle() {
        return (inputs.leftPosition.getDegrees() + inputs.rightPosition.getDegrees()) / 2;
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public void runCharacterization() {
        isChacterization = true;
        io.runVolts(voltage.getAsDouble());
    }

    public void endCharacterization() {
        isChacterization = false;
    }

    public void requestInterpolate() {
        setGoal(PivotGoal.INTERPOLATE);
    }

    public void requestStow() {
        setGoal(PivotGoal.STOW);
    }

    @AutoLogOutput(key = "Pivot/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
                        (inputs.leftPosition.getDegrees() + inputs.rightPosition.getDegrees()) / 2,
                        currentGoal.pivotAngle.getAsDouble(),
                        kPositionToleranceDegrees)
                || EqualsUtil.epsilonEquals(
                        inputs.leftPosition.getDegrees(),
                        currentGoal.pivotAngle.getAsDouble(),
                        kPositionToleranceDegrees)
                || EqualsUtil.epsilonEquals(
                        inputs.rightPosition.getDegrees(),
                        currentGoal.pivotAngle.getAsDouble(),
                        kPositionToleranceDegrees);
    }
}
