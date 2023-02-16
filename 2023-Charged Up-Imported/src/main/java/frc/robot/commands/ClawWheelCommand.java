package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.Constants;

public class ClawWheelCommand extends CommandBase {

  int _state;
  public ClawWheelCommand(int state) {
    _state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_state == 0) {
      ClawWheelsSubsystem.setIntakeSpeed(Constants.clawWheelForawrdSpeed);
    } else if (_state == 1) {
      ClawWheelsSubsystem.setIntakeSpeed(Constants.clawWheelReverseSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
     
}
