package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.Constants;
import frc.robot.OI;

public class ClawWheelCommand extends CommandBase {

  int _state;
  private ClawWheelsSubsystem clawWheelsSubsystem;
  public ClawWheelCommand(ClawWheelsSubsystem sub, int state) {
    _state = state;
    addRequirements(sub);
    clawWheelsSubsystem = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_state == 0) {
      clawWheelsSubsystem.setIntakeSpeed(1);
    } else {
      clawWheelsSubsystem.setIntakeSpeed(-0.5);
    }
    SmartDashboard.putNumber("R2 Axis", OI.getOperator().getR2Axis());
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
    /*
    if ( OI.getOperator().R2().getAsBoolean() == false || OI.getOperator().L2().getAsBoolean() == false) {
      return true;
    } else {
      return false;
    }
    */

  }
     
}
