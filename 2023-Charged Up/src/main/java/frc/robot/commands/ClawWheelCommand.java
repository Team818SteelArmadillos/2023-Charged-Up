package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants;

public class ClawWheelCommand extends CommandBase {

  int _state;
  private ClawSubsystem clawWheelsSubsystem;
  
  public ClawWheelCommand(int state, ClawSubsystem sub) {
    _state = state;
    clawWheelsSubsystem = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!clawWheelsSubsystem.inCubeMode()) {
      switch (_state) {
        case 0: clawWheelsSubsystem.setIntakeSpeed(Constants.CONE_IN_SPEED); break; //Cone in
        case 1: clawWheelsSubsystem.setIntakeSpeed(Constants.CONE_OUT_SPEED); break; //Cone out
      }
    } else {
      switch (_state) {
        case 0: clawWheelsSubsystem.setIntakeSpeed(Constants.CUBE_IN_SPEED); break; //Cube in
        case 1: clawWheelsSubsystem.setIntakeSpeed(Constants.CUBE_OUT_SPEED); break; //Cube out
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (_state == 0) {
      clawWheelsSubsystem.setIntakeSpeed(-0.1);
    } else {
      clawWheelsSubsystem.setIntakeSpeed(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;

  }
     
}
