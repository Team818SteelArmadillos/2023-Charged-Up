package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.PistonClawSubsystem;
import frc.robot.OI;

public class ClawCommand extends CommandBase {

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PistonClawSubsystem.setInkateReverse();
    ClawWheelsSubsystem.setIntakeSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.getOperator().rightTrigger()){

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
