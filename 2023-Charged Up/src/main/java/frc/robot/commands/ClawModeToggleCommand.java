package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawModeToggleCommand extends CommandBase {

  ClawSubsystem pistonClawSubsystem;
  int clawState;

  public ClawModeToggleCommand(ClawSubsystem sub1) {
    addRequirements(sub1);
    pistonClawSubsystem = sub1;
    clawState = 0;
  }

  public ClawModeToggleCommand(ClawSubsystem sub1, int clawState) {
    addRequirements(sub1);
    pistonClawSubsystem = sub1;
    this.clawState = clawState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (clawState) {
      case 1:
        pistonClawSubsystem.setClawOpen();
        break;
      case 2:
        pistonClawSubsystem.setClawClosed();
        break;
      default:
        pistonClawSubsystem.toggle();
        break;
    }
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