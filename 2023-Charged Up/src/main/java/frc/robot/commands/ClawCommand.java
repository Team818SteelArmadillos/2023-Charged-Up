package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.PistonClawSubsystem;

public class ClawCommand extends CommandBase {

  PistonClawSubsystem pistonClawSubsystem;

  public ClawCommand(PistonClawSubsystem sub) {
    addRequirements(sub);
    pistonClawSubsystem = sub;
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