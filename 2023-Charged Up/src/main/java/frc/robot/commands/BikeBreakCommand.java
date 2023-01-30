package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BikeBreakSubsystem;
import frc.robot.OI;

public class BikeBreakCommand extends CommandBase {

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BikeBreakSubsystem.setInkateReverse();
    BikeBreakSubsystem.setIntakeForward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
