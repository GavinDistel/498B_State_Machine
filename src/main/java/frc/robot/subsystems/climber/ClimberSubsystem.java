package frc.robot.subsystems.climber;

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
import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Ports;
import frc.robot.stateMachine.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX wheelMotor;

    private double wheelStatorCurrent;
    private boolean hasCage = false;

    private TalonFXConfiguration leftConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ClimberConstants.P).withKI(ClimberConstants.I)
                    .withKD(ClimberConstants.D).withKG(ClimberConstants.G).withGravityType(GravityTypeValue.Arm_Cosine))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((122.449 / 1.0)));
    private TalonFXConfiguration rightConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ClimberConstants.P).withKI(ClimberConstants.I)
                    .withKD(ClimberConstants.D).withKG(ClimberConstants.G).withGravityType(GravityTypeValue.Arm_Cosine))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((122.449 / 1.0)));
    private TalonFXConfiguration wheelConfig = new TalonFXConfiguration();

    private Follower right_motor_request = new Follower(Ports.ClimberPorts.LEFT_CLIMBER_MOTOR, true);
    private MotionMagicVoltage left_motor_request = new MotionMagicVoltage(0).withSlot(0);

    public ClimberSubsystem() {
        super(ClimberState.IDLE);
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.DeployMotionMagicCruiseVelocity;
        leftConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.DeployMotionMagicAcceleration;
        leftConfig.MotionMagic.MotionMagicJerk = ClimberConstants.MotionMagicJerk;
        rightConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.DeployMotionMagicCruiseVelocity;
        rightConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.DeployMotionMagicAcceleration;
        rightConfig.MotionMagic.MotionMagicJerk = ClimberConstants.MotionMagicJerk;
        wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.8;
        wheelConfig.CurrentLimits.StatorCurrentLimit = 60;
        wheelMotor = new TalonFX(Ports.ClimberPorts.CLIMBER_WHEEL_MOTOR);
        leftMotor = new TalonFX(Ports.ClimberPorts.LEFT_CLIMBER_MOTOR);
        rightMotor = new TalonFX(Ports.ClimberPorts.RIGHT_CLIMBER_MOTOR);
        wheelMotor.getConfigurator().apply(wheelConfig);
        leftMotor.getConfigurator().apply(leftConfig);
        rightMotor.getConfigurator().apply(rightConfig);
    }

    @Override
    protected void afterTransition(ClimberState newState) {
        switch (newState) {
            case IDLE -> {
                setHeadMotorSpeed(0.0);
                setWheelSpeed(ClimberSpeeds.IDLE);
            }
            case DEPLOYED -> {
                setClimberPosition(ClimberPositions.DEPLOYED);
                setWheelSpeed(ClimberSpeeds.DEPLOYED);
            }
            case CLIMBING -> {
                setClimberPosition(ClimberPositions.CLIMBING);
                setWheelSpeed(ClimberSpeeds.CLIMBING);
            }
        }
    }

    public void setHeadMotorSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void setWheelSpeed(double speed) {
        wheelMotor.set(speed);
    }

       @Override
    public void collectInputs(){
      //climberWheelSpeed = climberWheelMotor.get();
      wheelStatorCurrent = wheelMotor.getStatorCurrent().getValueAsDouble();
      DogLog.log(getName() + "/Climber wheel motor stator Current", wheelStatorCurrent);
      //DogLog.log(getName() + "/Climber wheel motor speed", climberWheelSpeed);
    }

    public boolean hasCage(){
      if (wheelStatorCurrent > Constants.ClimberConstants.cageStallCurrent){
        hasCage = true;
        return true;
      } else {
        hasCage = false;
        return false;
      } 
    }

    public boolean atGoal() {
        return true;
    }

    public void setClimberPosition(double climberSetpoint) {
        rightMotor.setControl(right_motor_request);
        leftMotor.setControl(left_motor_request.withPosition(climberSetpoint));
        DogLog.log(getName() + "/Left motor setpoint", climberSetpoint);
    }

    private static ClimberSubsystem instance;

    public static ClimberSubsystem getInstance() {
        if (instance == null)
            instance = new ClimberSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}