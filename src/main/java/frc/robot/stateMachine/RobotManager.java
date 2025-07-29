package frc.robot.stateMachine;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotManager extends StateMachine<RobotState> {
    public final IntakeSubsystem intake;
    public final ClimberSubsystem climber;

    public final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

    public RobotManager() {
        super(RobotState.IDLE);
        this.intake = IntakeSubsystem.getInstance();
        this.climber = ClimberSubsystem.getInstance();

    }

    @Override
    protected void collectInputs() {
    }

    @Override
    protected RobotState getNextState(RobotState currentState) {
        flags.log();
        RobotState nextState = currentState;
        for (RobotFlag flag : flags.getChecked()) {
            switch (flag) {
                case IDLE:
                    nextState = RobotState.PREPARE_IDLE;
                case INTAKE:
                    nextState = RobotState.PREPARE_INTAKE;
                    break;
                case CLIMB:
                    nextState = RobotState.CLIMBER_DEPLOYED;
                    break;
                case ROW1:
                    nextState = RobotState.PREPARE_SCORE_ROW1;
                    break;
                case ROW2:
                    nextState = RobotState.PREPARE_SCORE_ROW2;
                    break;
                case SCORE:
                    switch (nextState) {
                        case WAIT_SCORE_ROW1:
                            nextState = RobotState.SCORE_ROW1;
                            break;
                        case WAIT_SCORE_ROW2:
                            nextState = RobotState.SCORE_ROW2;
                            break;
                        case PREPARE_SCORE_ROW1:
                            nextState = RobotState.SCORE_ROW1;
                            break;
                        case PREPARE_SCORE_ROW2:
                            nextState = RobotState.SCORE_ROW2;
                            break;
                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }
        }

        switch (currentState) {
            case WAIT_SCORE_ROW1:
            case WAIT_SCORE_ROW2:
            case IDLE:
                break;

            case CLIMBER_DEPLOYED:
                if (ClimberSubsystem.getInstance().hasCage()) {
                    nextState = RobotState.CLIMBING;
                }
                break;
            case PREPARE_SCORE_ROW1:
                if (intake.atGoal()) {
                    nextState = RobotState.WAIT_SCORE_ROW1;
                }
                break;
            case PREPARE_SCORE_ROW2:
                if (intake.atGoal()) {
                    nextState = RobotState.WAIT_SCORE_ROW2;
                }
                break;
            case PREPARE_IDLE:
                if (intake.atGoal()) {
                    nextState = RobotState.IDLE;
                }
                break;
            case PREPARE_INTAKE:
                if (intake.atGoal()) {
                    nextState = RobotState.INTAKING;
                }
                break;
            case SCORE_ROW1:
                if (timeout(2)) {
                    nextState = RobotState.PREPARE_IDLE;
                }
                break;
            case SCORE_ROW2:
                if (timeout(2)) {
                    nextState = RobotState.PREPARE_IDLE;
                }
                break;
            case INTAKING:
                if (IntakeSubsystem.getInstance().hasCoral()) {
                    nextState = RobotState.PREPARE_IDLE;
                }
                break;
        }

        flags.clear();
        return nextState;

    }

    @Override
    protected void afterTransition(RobotState newState) {
        switch (newState) {
            case PREPARE_IDLE -> {
                intake.setStateFromRequest(IntakeState.IDLE);
            }
            case CLIMBER_DEPLOYED -> {
                climber.setStateFromRequest(ClimberState.DEPLOYED);
            }
            case CLIMBING -> {
                climber.setStateFromRequest(ClimberState.CLIMBING);
            }
            case CLIMBED -> {
                climber.setStateFromRequest(ClimberState.IDLE);
            }
            case PREPARE_SCORE_ROW1 -> {
                intake.setStateFromRequest(IntakeState.ROW1);
            }
            case SCORE_ROW1 -> {
                intake.setStateFromRequest(IntakeState.ROW1);
            }
            case PREPARE_SCORE_ROW2 -> {
                intake.setStateFromRequest(IntakeState.ROW2);
            }
            case SCORE_ROW2 -> {
                intake.setStateFromRequest(IntakeState.ROW2);
            }
            case PREPARE_INTAKE -> {
                intake.setStateFromRequest(IntakeState.INTAKE);
            }
            case INTAKING -> {
                intake.setStateFromRequest(IntakeState.INTAKE);
            }
            case IDLE,
                    WAIT_SCORE_ROW1,
                    WAIT_SCORE_ROW2 ->
                {
                }
        }
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void intakeRequest() {
        flags.check(RobotFlag.INTAKE);
    }

    public void scoreRequest() {
        flags.check(RobotFlag.SCORE);
    }

    public void climbRequest() {
        flags.check(RobotFlag.CLIMB);
    }

    public void row1Request() {
        flags.check(RobotFlag.ROW1);
    }

    public void row2Request() {
        flags.check(RobotFlag.ROW2);
    }

    public void idleRequest() {
        flags.check(RobotFlag.IDLE);
    }
}