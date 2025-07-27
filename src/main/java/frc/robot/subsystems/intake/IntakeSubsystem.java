package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.intakeConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
    public static TalonFX lMotor;
    public static TalonFX rMotor;
    public static TalonFX rollerMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(intakeConstants.P).withKI(intakeConstants.I)
                    .withKD(intakeConstants.D).withKG(intakeConstants.G)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((8.0357 / 1.0)));
    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private double IntakePosition;
    private final double tolerance;
    private double motorCurrent;
    private double motorStatorCurrent;
    private Follower right_motor_request = new Follower(Ports.IntakeWristPorts.lMotor, true);

    private MotionMagicVoltage left_motor_request = new MotionMagicVoltage(0).withSlot(0);

    public IntakeSubsystem() {
        super(IntakeState.IDLE);
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        lMotor = new TalonFX(Ports.IntakeWristPorts.lMotor);
        rMotor = new TalonFX(Ports.IntakeWristPorts.rMotor);
        rollerMotor = new TalonFX(Ports.IntakeRollersPorts.motor);
        motor_config.MotionMagic.MotionMagicCruiseVelocity = intakeConstants.MotionMagicCruiseVelocity;
        motor_config.MotionMagic.MotionMagicAcceleration = intakeConstants.MotionMagicAcceleration;
        motor_config.MotionMagic.MotionMagicJerk = intakeConstants.MotionMagicJerk;
        lMotor.getConfigurator().apply(motor_config);
        rMotor.getConfigurator().apply(motor_config);
        rollerMotor.getConfigurator().apply(rollerConfig);
        tolerance = 0.1;
    }

    protected IntakeState getNexState(IntakeState currentState) {
        return currentState;
    }

    public boolean atGoal() {
        return switch (getState()) {
            case IDLE ->
                MathUtil.isNear(IntakePositions.IDLE, IntakePosition, tolerance);
            case INTAKE ->
                MathUtil.isNear(IntakePositions.INTAKE, IntakePosition, tolerance);
            case ROW1 ->
                MathUtil.isNear(IntakePositions.ROW_1, IntakePosition, tolerance);
            case ROW2 ->
                MathUtil.isNear(IntakePositions.ROW_2, IntakePosition, tolerance);
        };

    }

    public void increaseSetpoint(){
        System.out.println("Setpoints increased to " + (IntakePositions.INTAKE + 0.005));
        switch (getState()) {
          case ROW1 -> {
            IntakePositions.ROW_1 += 0.005;
            setIntakePosition(IntakePositions.ROW_1);
            break;
          }
          case ROW2 -> {
            IntakePositions.ROW_2 += 0.005;
            setIntakePosition(IntakePositions.ROW_2);
            break;
          }
          case INTAKE-> {
            IntakePositions.INTAKE += 0.005;
            setIntakePosition(IntakePositions.INTAKE);
            break;
          }
        }
      }

    public void decreaseSetpoint() {
        System.out.println("Setpoints decreased to " + (IntakePositions.INTAKE - 0.005));
        switch (getState()) {
            case ROW1 -> {
                IntakePositions.ROW_1 -= 0.005;
                setIntakePosition(IntakePositions.ROW_1);
                break;
            }
            case ROW2 -> {
                IntakePositions.ROW_2 -= 0.005;
                setIntakePosition(IntakePositions.ROW_2);
                break;
            }
            case INTAKE -> {
                IntakePositions.INTAKE -= 0.005;
                setIntakePosition(IntakePositions.INTAKE);
                break;
            }
        }
    }

      @Override
  public void collectInputs() {
    IntakePosition = lMotor.getPosition().getValueAsDouble();
    motorCurrent = lMotor.getStatorCurrent().getValueAsDouble();
    DogLog.log(getName() + "/Intake Position", IntakePosition);
    //DogLog.log(getName() + "/Intake wrist motor current", motorCurrent);
    //DogLog.log(getName() + "/Intake wrist AtGoal", atGoal());
  }

  public boolean hasCoral(){
    return motorStatorCurrent > intakeConstants.stallCurrent;
  }

  @Override
  public void periodic() {
    // System.out.println(encoder.get());ph
    super.periodic();
    }

    public void setIntakePosition(double position) {
        rMotor.setControl(right_motor_request);
        lMotor.setControl(left_motor_request.withPosition(position));
        // DogLog.log(getName() + "/Elbow Setpoint", position);
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    protected void afterTransition(IntakeState newState) {
      switch (newState) {
        case IDLE -> {
          setIntakePosition(IntakePositions.IDLE);
          setRollerSpeed(IntakeSpeeds.IDLE);
        }
        case INTAKE-> {
          setIntakePosition(IntakePositions.INTAKE);
          setRollerSpeed(IntakeSpeeds.INTAKE);
        }
        case ROW1 -> {
          setIntakePosition(IntakePositions.ROW_1);
          setRollerSpeed(IntakeSpeeds.ROW_1);
        }
        case ROW2 -> {
          setIntakePosition(IntakePositions.ROW_2);
          setRollerSpeed(IntakeSpeeds.ROW_2);
        }
      }
    }

    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) instance = new IntakeSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}