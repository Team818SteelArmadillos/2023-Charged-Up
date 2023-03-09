package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.Constants;
import frc.robot.OI;

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
      ClawWheelsSubsystem.setIntakeSpeed( OI.getOperator().getR2Axis());
    } else {
      ClawWheelsSubsystem.setIntakeSpeed( -(OI.getOperator().getL2Axis()) );
    }
    SmartDashboard.putNumber("R2 Axis", OI.getOperator().getR2Axis());
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
