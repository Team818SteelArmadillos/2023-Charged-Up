package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PistonClawSubsystem;

public class ClawCommand extends CommandBase {

  public static int _state;
  public ClawCommand(int state) {
    _state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (PistonClawSubsystem.isOpen() == false) { 
      PistonClawSubsystem.setClawOpen();
    } else {
      PistonClawSubsystem.setClawClosed();
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