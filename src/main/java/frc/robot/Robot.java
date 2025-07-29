package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotCommands;
import frc.robot.stateMachine.RobotManager;
import frc.robot.stateMachine.RobotState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;

import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends TimedRobot {
  // public static final double DEFAULT_PERIOD = 0.02;
  // public final Timer setupTimer = new Timer();
  // public double setupTime = 0;

  public static RobotManager robotManager = new RobotManager();
  public static RobotCommands robotCommands = new RobotCommands(robotManager);
  // public static int coordinateFlip = 1;
  // public static int rotationOffset = 0;

  public static Optional<Alliance> alliance = Optional.empty();
  public static final Controls controls = new Controls();

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Command> newAutoChooser;

  @Override
  public void robotInit() {
    controls.configureDefaultCommands();
    controls.configureDriverCommands();
    controls.configureOperatorCommands();

    NamedCommands.registerCommand("score", robotCommands.scoreCommand());
    NamedCommands.registerCommand("intake", robotCommands.intakeCommand());
    NamedCommands.registerCommand("return to idle", robotCommands.idleCommand());
    CommandSwerveDrivetrain.getInstance();

    // newAutoChooser = AutoBuilder.buildAutoChooser("1CoralBlindMiddle");
    newAutoChooser = new SendableChooser<Command>();
    newAutoChooser.addOption("1CoralBlindMiddle", new PathPlannerAuto("1CoralBlindMiddle"));
    // newAutoChooser = AutoBuilder.buildAutoChooser("1CoralBlindMiddle");
    newAutoChooser.addOption("processor 2.5 coral", new PathPlannerAuto("2.5 Coral Blind Processor Side"));
    newAutoChooser.addOption("non-processor 2.5 coral", new PathPlannerAuto("2.5 Coral Blind Non-Processor Side"));

    autoChooser = AutoBuilder.buildAutoChooser();

    // Limelight.getInstance();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(newAutoChooser);
    if (alliance.isEmpty()) {
      alliance = DriverStation.getAlliance();
    }
  }

  @Override
  public void disabledPeriodic() {
    alliance = DriverStation.getAlliance();
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    controls.driver.rumble(0);
  }

  @Override
  public void disabledInit() {
    // setupTimer.restart();
    // drivetrain.enableBrakeMode(false);
  }

  Timer timer = new Timer();
  boolean hasFired;

  @Override
  public void autonomousInit() {
    timer.start();
    // new FullScore().schedule();
    // drivetrain.enableBrakeMode(true);
    // matchStarted = true;
    IntakeSubsystem.getInstance();
    hasFired = false;

    // if (autoToRun == null)
    // autoToRun = defaultAuto;
    autoChooser.getSelected().schedule();
    // if(newAutoChooser.getSelected() != null){
    // newAutoChooser.getSelected().schedule();
    // }
    // autoToRun = new HighHighCone();

    // if (alliance.get() == Alliance.Blue) {
    // CommandSwerveDrivetrain.getInstance().(autoToRun.getInitialPose().getRotation().getDegrees());
    // Drivetrain.getInstance().setPose(autoToRun.getInitialPose());
    // } else {
    // Drivetrain.getInstance().setYaw(PoseUtil.flipAngleDegrees(autoToRun.getInitialPose().getRotation().getDegrees()));
    // Drivetrain.getInstance().setPose(PoseUtil.flip(autoToRun.getInitialPose()));
    // }
    // SmartDashboard.putData((Sendable) autoToRun.getInitialPose());

    // autoToRun.getCommand().schedule();
    // new LongTaxi().getCommand().schedule();

    // CommandScheduler.getInstance().run();

    // if (alliance.get() == Alliance.Blue) {
    // Drivetrain.getInstance().setYaw(autoToRun.getInitialPose().getRotation().getDegrees());
    // Drivetrain.getInstance().setPose(autoToRun.getInitialPose());
    // } else {
    // Drivetrain.getInstance().setYaw(PoseUtil.flip(autoToRun.getInitialPose()).getRotation().getDegrees());
    // Drivetrain.getInstance().setPose(PoseUtil.flip(autoToRun.getInitialPose()));
    // }
    // Sets the LEDs to a pattern for auto, this could be edited to include code for
    // if vision is aligned for auto diagnosis
  }

  @Override
  public void autonomousPeriodic() {
    // System.out.println(timer.get());
    // if(timer.get() > 5 && timer.get() < 7){
    // if(!hasFired){
    // IntakeWristSubsystem.getInstance().setIntakePosition(.15);
    // // robotCommands.L1Row1Command();
    // IntakeRollersSubsystem.getInstance().setIntakeRollerSpeeds(.2);
    // hasFired = true;
    // }
    // }else if(timer.get() > 7){
    // if(hasFired){
    // // robotCommands.idleCommand();
    // IntakeWristSubsystem.getInstance().setIntakePosition(.15);
    // // robotCommands.L1Row1Command();
    // IntakeRollersSubsystem.getInstance().setIntakeRollerSpeeds(0);
    // hasFired = false;
    // CommandScheduler.getInstance().cancelAll();;
    // }
    // }else if(timer.get() > 9){
    // if(!hasFired){
    // CommandScheduler.getInstance().run();
    // hasFired = true;
    // }
    // }

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}