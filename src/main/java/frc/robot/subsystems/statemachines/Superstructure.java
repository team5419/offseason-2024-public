package frc.robot.subsystems.statemachines;

import frc.robot.RobotContainer;
import frc.robot.lib.VirtualSubsystem;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends VirtualSubsystem {

    public enum Request {
        REQ_IDLE,
        REQ_SHOOT,
        REQ_INTAKE,
        REQ_INTERPOLATE,
        REQ_STOW,
        // add more requests here (they will be requested by the drivers)
    }

    // we will likely end up creating different enum sets that are specific
    // for common checks but this is here for now as an example
    private static Set<Request> allRequests = EnumSet.allOf(Request.class);

    @Getter
    private SuperState currentState = SuperState.IDLE;

    private Request currentRequest;

    private final IntakeStateMachine intakeStateMachine;
    private final Pivot pivot;

    private PivotGoal pivotShootGoal;

    public enum SuperState {
        IDLE(allRequests) {
            @Override
            public void actions(Superstructure s) {}

            @Override
            public SuperState transitions(Superstructure s) {
                SuperState curState = s.currentState;
                return switch (s.currentRequest) {
                    case REQ_IDLE -> this;
                    case REQ_SHOOT -> curState.toShoot(s);
                    case REQ_INTAKE -> s.pivot.atGoal() ? curState.toIntake(s) : this;
                    case REQ_INTERPOLATE -> curState.toInterpolate(s);
                    case REQ_STOW -> s.pivot.atGoal() ? this : s.currentState.toStow(s);
                    default -> this;
                };
            }
        },
        INTAKE(allRequests) {
            @Override
            public void actions(Superstructure s) {
                s.intakeStateMachine.requestIntake();
            }

            @Override
            public SuperState transitions(Superstructure s) {
                SuperState curState = s.currentState;
                return switch (s.currentRequest) {
                    case REQ_IDLE -> curState.toIdle(s);
                    case REQ_INTAKE -> this;
                    case REQ_SHOOT -> s.intakeStateMachine.noteReady() ? curState.toShoot(s) : this;
                    case REQ_INTERPOLATE -> curState.toInterpolate(s); // might need check for pivot restraints
                    case REQ_STOW -> curState.toStow(s);
                    default -> throw new AssertionError("Unknown request: " + s.currentRequest);
                };
            }
        },
        SHOOT(allRequests) {
            @Override
            public void actions(Superstructure s) {
                s.pivot.setGoal(s.pivotShootGoal);
            }

            @Override
            public SuperState transitions(Superstructure s) {
                SuperState curState = s.currentState;
                return switch (s.currentRequest) {
                    case REQ_IDLE -> curState.toIdle(s);
                    case REQ_INTAKE -> s.intakeStateMachine.noteInSystem() ? this : curState.toStow(s);
                    case REQ_SHOOT -> s.intakeStateMachine.noteReady() ? this : curState.toStow(s);
                    case REQ_INTERPOLATE -> s.intakeStateMachine.noteInSystem() ? this : curState.toInterpolate(s);
                    case REQ_STOW -> curState.toStow(s);
                    default -> s.intakeStateMachine.noteInSystem() ? this : curState.toStow(s);
                };
            }
        },
        INTERPOLATE(allRequests) {
            @Override
            public void actions(Superstructure s) {}

            @Override
            public SuperState transitions(Superstructure s) {
                SuperState curState = s.currentState;
                return switch (s.currentRequest) {
                    case REQ_IDLE, REQ_STOW -> curState.toStow(s);
                    case REQ_INTAKE -> curState.toIntake(s);
                    default -> this;
                };
            }
        },
        STOW(allRequests) {
            @Override
            public void actions(Superstructure s) {
                s.pivot.setGoal(PivotGoal.STOW);
            }

            @Override
            public SuperState transitions(Superstructure s) {
                SuperState curState = s.currentState;
                return switch (s.currentRequest) {
                    case REQ_INTAKE -> curState.toIntake(s);
                    case REQ_SHOOT, REQ_STOW -> this;
                    case REQ_INTERPOLATE -> curState.toInterpolate(s);
                    default -> (s.pivot.atGoal() && (s.pivot.getCurrentGoal() == PivotGoal.STOW))
                            ? curState.toIdle(s)
                            : this;
                };
            }
        };

        // add more states here

        public Set<Request> requests;

        private SuperState(Set<Request> requests) {
            this.requests = requests;
        }

        private SuperState(Request... requests) {
            this(EnumSet.copyOf(Arrays.asList(requests)));
        }

        // All of these following methods -- to___ -- are assuming all the checks have been done in the transition,
        // and this is just doing the things to change ex requests

        private SuperState toIdle(Superstructure s) {
            s.currentRequest = Request.REQ_IDLE;
            s.intakeStateMachine.requestIdle();
            return SuperState.IDLE;
        }

        private SuperState toShoot(Superstructure s) {
            boolean shouldShoot = s.intakeStateMachine.noteInSystem() || s.intakeStateMachine.noteReady();
            s.intakeStateMachine.requestShoot();
            // if (shouldShoot) s.intakeStateMachine.requestShoot();
            // else s.intakeStateMachine.requestIntake();
            return shouldShoot ? SuperState.SHOOT : SuperState.INTAKE;
        }

        private SuperState toIntake(Superstructure s) {
            s.intakeStateMachine.requestIntake();
            return SuperState.INTAKE;
        }

        private SuperState toInterpolate(Superstructure s) {
            s.intakeStateMachine.requestInterpolate();
            s.pivot.requestInterpolate();
            return SuperState.INTERPOLATE;
        }

        private SuperState toStow(Superstructure s) {
            s.intakeStateMachine.requestIdle();
            s.pivot.requestStow();
            return SuperState.STOW;
        }

        /**
         * Called from the Superstructure periodic method to get the next state.
         */
        public final SuperState periodic(Superstructure s) {
            actions(s);
            return this.transitions(s);
        }

        /**
         * Actions always performed in periodic when we are in this state that
         * are independent of the current request and do not affect what the
         * next state will be.
         */
        protected void actions(Superstructure s) {}

        /**
         * <p>
         * May have side effects on the Superstructure before returning the state.
         * This implementation provides the default transitions but a given
         * state can easily override how it handles particular requests
         * by checking for them and returning a different next state.
         * </p>
         *
         * <p>
         * Note that this method is only called for requests that the state has
         * in its `handles` list. So a state has three choices how to deal with
         * a given request
         * </p>
         *
         *  <p>
         *  1. Leave it out of its handles list and the state will not change
         *     (though the `actions` method will still be called.)
         * </p>
         *
         *  <p>
         *  2. Put it in its handles list but don't explicitly handle it,
         *     delegating to super.nextState() to get the default handling.
         * </p>
         *
         *  <p>
         *  3. Put in in the handles list and explicitly handle it rather than
         *     calling super.nextState(). This is useful for cases where we need
         *     to do something before returning the new state or possibly if we
         *     want to transition to a different state though it's not clear
         *     that the latter ever happens.
         * </p>
         *
         * <p>
         * Some states will handle some (or most) requests by ignoring
         * them and just returning `this`. That is a valid version of #3.
         * </p>
         *
         * <p>
         ** TLDR:
         ** Handles transitions to the next state based on the request
         * </p>
         * @return The state of the superstructure in the next periodic loop
         */
        protected SuperState transitions(Superstructure s) {
            return this;
        }
        ;
    }

    public Superstructure(RobotContainer container) {
        currentRequest = Request.REQ_IDLE;
        intakeStateMachine = container.getIntakeStateMachine();
        // intake = container.getIntake();
        // flywheels = container.getFlywheels();
        pivot = container.getPivot();
        pivotShootGoal = PivotGoal.SUBWOOFER;
    }

    private void setRequest(Request request) {
        currentRequest = request;
    }

    public void requestIdle() {
        setRequest(Request.REQ_IDLE);
    }

    public void requestIntake() {
        setRequest(Request.REQ_INTAKE);
    }

    public void requestShoot() {
        setRequest(Request.REQ_SHOOT);
    }

    public void requestInterpolate() {
        setRequest(Request.REQ_INTERPOLATE);
    }

    public void requestStow() {
        setRequest(Request.REQ_STOW);
    }

    public void setPivotShootGoal(PivotGoal pivotGoal) {
        pivotShootGoal = pivotGoal;
        Leds.getInstance().isStockpile = pivotGoal == PivotGoal.STOCKPILE;
    }

    @Override
    public void periodic() {
        currentState = currentState.periodic(this);
        Logger.recordOutput("Superstructure/Current Request", currentRequest);
        Logger.recordOutput("Superstructure/Current State", currentState);
    }
}
