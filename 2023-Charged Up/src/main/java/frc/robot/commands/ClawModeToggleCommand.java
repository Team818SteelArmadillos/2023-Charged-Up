package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawModeToggleCommand extends CommandBase {

  ClawSubsystem pistonClawSubsystem;

  public ClawModeToggleCommand(ClawSubsystem sub1) {
    addRequirements(sub1);
    pistonClawSubsystem = sub1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pistonClawSubsystem.toggle();
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
    return true;
  }
     
}