package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.lib.util.EqualsUtil;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // this subsystem also controls the indexer motor
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

    private static final LoggedTunableNumber velocityRPS = new LoggedTunableNumber("Intake/Target Velocity RPS", 40.0);
    // private static final LoggedTunableNumber voltage = new LoggedTunableNumber("Intake/Voltage", 0.5);

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", gains.kS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", gains.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/kA", gains.kA());

    private static final LoggedTunableNumber intakeRPS = new LoggedTunableNumber("Intake/Intake RPS Goal", 50);
    private static final LoggedTunableNumber indexRPS = new LoggedTunableNumber("Intake/Index RPS Goal", 10);
    private static final LoggedTunableNumber outtakeRPS = new LoggedTunableNumber("Intake/Outtake RPS Goal", -10);
    private static final LoggedTunableNumber shootRPS = new LoggedTunableNumber("Intake/Shoot RPS Goal", 20);

    public enum IntakeGoal {
        IDLE(() -> 0),
        INTAKE(intakeRPS),
        INDEX(indexRPS),
        OUTTAKE(outtakeRPS),
        SHOOT(shootRPS);

        @Getter
        private DoubleSupplier intakeVelocity;

        private IntakeGoal(DoubleSupplier intakeVelocity) {
            this.intakeVelocity = intakeVelocity;
        }
    }

    @AutoLogOutput(key = "Intake/Intake Goal")
    private IntakeGoal currentGoal = IntakeGoal.IDLE;

    public Intake(IntakeIO io) {
        this.io = io;

        setDefaultCommand(runOnce(() -> setGoal(IntakeGoal.IDLE)).withName("Intake Idle"));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Check controllers
        LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(), sva -> ff = new SimpleMotorFeedforward(sva[0], sva[1], sva[2]), kS, kV, kA);

        if (DriverStation.isDisabled()) {
            setGoal(IntakeGoal.IDLE);
        }

        if (currentGoal == IntakeGoal.IDLE && atGoal()) stop();
        else runVelocity(currentGoal.intakeVelocity.getAsDouble());
    }

    public void setGoal(IntakeGoal desiredGoal) {
        currentGoal = desiredGoal;
    }

    @AutoLogOutput(key = "Intake/At Goal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(
                (inputs.leftMotorVelocityRPS + inputs.rightMotorVelocityRPS) / 2,
                currentGoal.getIntakeVelocity().getAsDouble(),
                kVelocityTolerance);
    }

    public void runVelocity() {
        io.runVelocity(velocityRPS.getAsDouble(), ff.calculate(velocityRPS.getAsDouble()));
    }

    public void runVelocity(double velocity) {
        io.runVelocity(velocity, ff.calculate(velocityRPS.getAsDouble()));
    }

    public void stop() {
        io.stop();
    }
}
