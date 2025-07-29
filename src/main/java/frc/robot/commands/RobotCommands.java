package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.stateMachine.RobotManager;
import frc.robot.stateMachine.RobotState;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList = List.of(robot.intake, robot.climber);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command scoreCommand() {
    return Commands.runOnce(robot::scoreRequest, requirements);
    // .andThen(robot.waitForState(RobotState.SCORE_L1));
  }

  public Command idleCommand() {
    return Commands.runOnce(robot::idleRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE));
  }

  public Command intakeCommand() {
    return Commands.runOnce(robot::intakeRequest, requirements);
    // /.andThen(robot.waitForState(RobotState.INTAKE));
  }

  public Command Row1Command() {
    return Commands.runOnce(robot::row1Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_SCORE_ROW1));
  }

  public Command Row2Command() {
    return Commands.runOnce(robot::row2Request, requirements)
        .andThen(robot.waitForState(RobotState.WAIT_SCORE_ROW2));
  }

  public Command climbCommand() {
    return (Commands.runOnce(robot::climbRequest, requirements))
        .andThen(robot.waitForState(RobotState.CLIMBER_DEPLOYED));
  }

}