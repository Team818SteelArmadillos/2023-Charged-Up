package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.Constants;
import frc.robot.OI;

public class ClawWheelCommand extends CommandBase {

  int _state;
  private ClawWheelsSubsystem clawWheelsSubsystem;
  
  public ClawWheelCommand(int state, ClawWheelsSubsystem sub) {
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
    switch (_state) {
      case 0: clawWheelsSubsystem.setIntakeSpeed(Constants.clawWheelForawrdSpeed); break;
      case 1: clawWheelsSubsystem.setIntakeSpeed(Constants.clawWheelReverseSpeed); break;
      case 2: clawWheelsSubsystem.setIntakeSpeed(Constants.clawWheelReverseSpeed / 4); break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawWheelsSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;

  }
     
}
