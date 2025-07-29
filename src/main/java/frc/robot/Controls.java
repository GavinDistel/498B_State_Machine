package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.OIConstants;
import frc.robot.drivers.Xbox;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Controls {
    private double maxSpeed = TunerConstants.kSpeedAt12Volts;
    private final double turtleSpeed = 0.1;
    private double maxAngularRate = Math.PI * 3.5;
    private final double turtleAngularRate = Math.PI * 0.5;
    private double angularRate = maxAngularRate;
    private double drivetrainSpeed = 0.75;

    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(maxSpeed * 0.1) // Deadband is handled on input
            .withRotationalDeadband(angularRate * 0.1);

    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    public Controls() {
        driver.setDeadzone(0.15);
        driver.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
        operator.setTriggerThreshold(0.2);
    }

    private Supplier<SwerveRequest> controlStyle;

    private void newControlStyle() {
        controlStyle = () -> drive
                .withVelocityX((-(driver.leftY() * .5) * (driver.leftY() * .5) * (driver.leftY() * .5) * maxSpeed) * .7) // Drive
                                                                                                                         // forward
                                                                                                                         // -Y
                .withVelocityY((-(driver.leftX() * .5) * (driver.leftX() * .5) * (driver.leftX() * .5) * maxSpeed) * .7) // Drive
                                                                                                                         // left
                                                                                                                         // with
                                                                                                                         // negative
                                                                                                                         // X
                                                                                                                         // (left)
                .withRotationalRate((driver.rightX() * angularRate) * .1); // Drive counterclockwise with negative X
                                                                           // (left)
    }

    public void configureDefaultCommands() {
        newControlStyle();
        CommandSwerveDrivetrain.getInstance().setDefaultCommand(repeatingSequence( // Drivetrain will execute this
                                                                                   // command periodically
                runOnce(() -> CommandSwerveDrivetrain.getInstance()
                        .driveFieldRelative(new ChassisSpeeds(
                                -(driver.leftY() * drivetrainSpeed) * (driver.leftY()) * (driver.leftY()) * maxSpeed,
                                -(driver.leftX() * drivetrainSpeed) * (driver.leftX()) * (driver.leftX()) * maxSpeed,
                                driver.rightX() * angularRate)),
                        CommandSwerveDrivetrain.getInstance())));
    }

    public void decreaseSpeeds() {
        angularRate = Math.PI * 1;
        drivetrainSpeed = .3;
    }

    public void normalizeSpeeds() {
        angularRate = Math.PI * 3.5;
        drivetrainSpeed = .6;
    }

    public void configureDriverCommands() {
        driver.A().onTrue(runOnce(() -> CommandSwerveDrivetrain.getInstance().setYaw(Robot.alliance.get())));
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand());
        driver.rightTrigger().onFalse(Robot.robotCommands.idleCommand());

        driver.leftTrigger().onTrue(Robot.robotCommands.intakeCommand());
        driver.leftTrigger().onFalse(Robot.robotCommands.idleCommand());
        driver.X().whileTrue(runOnce(() -> drivetrain.applyRequest(() -> drivetrain.brake)));
        driver.Y().whileTrue(drivetrain.applyRequest(() -> drivetrain.point
                .withModuleDirection(new Rotation2d(-(driver.leftY() * .5), -(driver.leftX() * .5)))));
        driver.POV0().onTrue(Robot.robotCommands.climbCommand());
        driver.rightBumper().onTrue(runOnce(() -> decreaseSpeeds()));
        driver.rightBumper().onFalse(runOnce(() -> normalizeSpeeds()));
    }

    public void configureOperatorCommands() {
        operator.Y().onTrue(Robot.robotCommands.Row1Command());
        operator.A().onTrue(Robot.robotCommands.Row2Command());
        operator.leftBumper().onTrue(Robot.robotCommands.idleCommand());
        operator.POV90().onTrue(runOnce(() -> IntakeSubsystem.getInstance().increaseSetpoint()));
        operator.POVMinus90().onTrue(runOnce(() -> IntakeSubsystem.getInstance().decreaseSetpoint()));
    }
}
