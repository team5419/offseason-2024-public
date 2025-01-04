package frc.robot.subsystems.statemachines;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Ports;
import frc.robot.lib.RumbleThread;
import frc.robot.lib.RumbleThread.ControllersToRumble;
import frc.robot.lib.VirtualSubsystem;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.Flywheels.FlywheelsGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class IntakeStateMachine extends VirtualSubsystem {

    public enum Request {
        // NONE,
        REQ_IDLE,
        REQ_INTAKE,
        REQ_SHOOT,
        REQ_INTERPOLATE
        // add more requests here (they will be requested by the Superstructure)
    }

    // we will likely end up creating different enum sets that are specific
    // for common checks but this is here for now as an example
    private static Set<Request> allRequests = EnumSet.allOf(Request.class);

    private Request currentRequest;

    @Getter
    private IntakeState currentState = IntakeState.IDLE;

    private Intake intake;
    private Flywheels flywheels;
    private Pivot pivot; // very silly that its here but we'll fix later
    private Swerve swerve;

    private DigitalInput indexerBeambreak, flywheelsBeambreak;

    public enum IntakeState {
        IDLE(allRequests) {

            public void actions(IntakeStateMachine intakeSM) {
                intakeSM.currentState.setGoals(intakeSM, IntakeGoal.IDLE, FlywheelsGoal.IDLE);
            }

            public IntakeState transitions(IntakeStateMachine intakeSM) {
                return switch (intakeSM.currentRequest) {
                    case REQ_INTAKE, REQ_INTERPOLATE -> INTAKE;
                    case REQ_SHOOT -> intakeSM.noteReady() ? SHOOT : this;
                    default -> this;
                };
            }
        },
        INTAKE(allRequests) {

            public void actions(IntakeStateMachine intakeSM) {
                intakeSM.currentState.setGoals(intakeSM, IntakeGoal.INTAKE, FlywheelsGoal.INTAKE);
            }

            public IntakeState transitions(IntakeStateMachine intakeSM) {
                return switch (intakeSM.currentRequest) {
                    case REQ_IDLE -> IDLE;
                    case REQ_INTAKE -> intakeSM.noteInSystem() ? IntakeState.INDEX : this;
                    default -> this; // not too sure about this (haha lol "this" in both senses im so funny ._.)
                };
            }
        },
        INDEX(allRequests) {

            public void actions(IntakeStateMachine intakeSM) {
                if (intakeSM.noteInSystem()) {
                    RumbleThread.getInstance().setRumble(RumbleType.kBothRumble, 0.75, 0.5, ControllersToRumble.ALL);
                    Leds.getInstance().hasNote = true;
                }
                intakeSM.currentState.setGoals(intakeSM, IntakeGoal.INDEX, FlywheelsGoal.IDLE);
            }

            public IntakeState transitions(IntakeStateMachine intakeSM) {
                return switch (intakeSM.currentRequest) {
                    case REQ_IDLE -> IDLE;
                    case REQ_INTAKE -> intakeSM.noteReady() ? CENTER : this;
                    case REQ_SHOOT -> intakeSM.noteReady() ? SHOOT : this;
                    default -> intakeSM.noteInSystem() ? this : IDLE;
                };
            }
        },
        CENTER(allRequests) {
            public void actions(IntakeStateMachine intakeSM) {
                intakeSM.currentState.setGoals(intakeSM, IntakeGoal.IDLE, FlywheelsGoal.IDLE);
            }

            public IntakeState transitions(IntakeStateMachine intakeSM) {
                return switch (intakeSM.currentRequest) {
                    case REQ_IDLE -> IDLE;
                    case REQ_SHOOT -> SHOOT;
                    case REQ_INTAKE -> intakeSM.noteReady() && intakeSM.noteInSystem() ? this : INTAKE;
                    default -> intakeSM.noteReady() ? this : IDLE;
                };
            }
        },
        SHOOT(allRequests) {

            public void actions(IntakeStateMachine intakeSM) {
                intakeSM.currentState.setFlywheelsGoal(intakeSM, FlywheelsGoal.SHOOT);
                if (intakeSM.flywheels.atGoal() && intakeSM.pivot.atGoal()) {
                    intakeSM.currentState.setIntakeGoal(intakeSM, IntakeGoal.SHOOT);
                    Leds.getInstance().hasNote = false;
                }
            }

            public IntakeState transitions(IntakeStateMachine intakeSM) {
                return switch (intakeSM.currentRequest) {
                    case REQ_IDLE -> IDLE;
                    case REQ_INTAKE -> intakeSM.noteInSystem() ? this : INTAKE;
                    default -> intakeSM.noteReady() ? this : IDLE;
                };
            }
        },
        INTERPOLATE(allRequests) {

            public void actions(IntakeStateMachine intakeSM) {
                intakeSM.pivot.setGoal(PivotGoal.INTERPOLATE);
                intakeSM.flywheels.setGoal(FlywheelsGoal.INTERPOLATE);
                intakeSM.swerve.setHeadingGoal(() -> RobotState.getInstance()
                        .getVelocityCompensatedYaw(
                                AllianceFlipUtil.apply(FieldConstants.Speaker.kCenterSpeakerOpeningPose)));
            }

            public IntakeState transitions(IntakeStateMachine intakeSM) {
                return switch (intakeSM.currentRequest) {
                    case REQ_IDLE -> intakeSM.clearToState(IDLE);
                    default -> (intakeSM.noteReady() && intakeSM.flywheels.atGoal() && intakeSM.pivot.atGoal())
                            ? intakeSM.clearToState(SHOOT)
                            : this;
                };
            }
        };

        // add more states here

        public Set<Request> requests;

        private IntakeState(Set<Request> requests) {
            this.requests = requests;
        }

        private IntakeState(Request... requests) {
            this(EnumSet.copyOf(Arrays.asList(requests)));
        }

        private void setGoals(IntakeStateMachine intakeSM, IntakeGoal intakeG, FlywheelsGoal flyG) {
            intakeSM.intake.setGoal(intakeG);
            intakeSM.flywheels.setGoal(flyG);
        }

        private void setIntakeGoal(IntakeStateMachine intakeSM, IntakeGoal intakeG) {
            intakeSM.intake.setGoal(intakeG);
        }

        private void setFlywheelsGoal(IntakeStateMachine intakeSM, FlywheelsGoal flyG) {
            intakeSM.flywheels.setGoal(flyG);
        }

        /**
         * Called from the Superstructure periodic method to get the next state.
         */
        public final IntakeState periodic(IntakeStateMachine intakeSM) {
            actions(intakeSM);
            return this.transitions(intakeSM);
        }

        /**
         * Actions always performed in periodic when we are in this state that
         * are independent of the current request and do not affect what the
         * next state will be.
         */
        protected void actions(IntakeStateMachine intakeStateMachine) {}

        /**
         * Handles transitions to the next state based on the request
         * @return The state of the intake state machine in the next periodic loop
         */
        protected IntakeState transitions(IntakeStateMachine intakeStateMachine) {
            return this;
        }
    }

    public IntakeStateMachine(Intake intake, Flywheels flywheels, Pivot pivot, Swerve swerve) {
        currentRequest = DriverStation.isAutonomous() ? Request.REQ_INTAKE : Request.REQ_IDLE;
        this.intake = intake;
        this.flywheels = flywheels;
        this.pivot = pivot;
        this.swerve = swerve;
        indexerBeambreak = new DigitalInput(Ports.kIndexerBeamBreakID);
        flywheelsBeambreak = new DigitalInput(Ports.kFlywheelsBeamBreakID);
    }

    public boolean noteInSystem() {
        return !indexerBeambreak.get();
    }

    public boolean noteReady() {
        return !flywheelsBeambreak.get();
    }

    private void setRequest(Request request) {
        currentRequest = request;
    }

    @Override
    public void periodic() {
        currentState = currentState.periodic(this);
        Logger.recordOutput("Intake State Machine/Current request", currentRequest);
        Logger.recordOutput("Intake State Machine/Current state", currentState);
        Logger.recordOutput("Intake State Machine/Indexer beam break", noteInSystem());
        Logger.recordOutput("Intake State Machine/Flywheels beam break", noteReady());
    }

    private IntakeState clearToState(IntakeState state) {
        swerve.clearHeadingGoal();
        return state;
    }

    // requests

    public void requestIdle() {
        setRequest(Request.REQ_IDLE);
    }

    public void requestShoot() {
        setRequest(Request.REQ_SHOOT);
    }

    public void requestIntake() {
        setRequest(Request.REQ_INTAKE);
    }

    public void requestInterpolate() {
        setRequest(Request.REQ_INTERPOLATE);
    }
}
