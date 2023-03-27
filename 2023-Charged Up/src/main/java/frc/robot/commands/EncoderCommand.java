package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class EncoderCommand extends CommandBase {

  private ArmSubsystem armSubsystem;

  public EncoderCommand(ArmSubsystem sub) {
    addRequirements(sub);
    armSubsystem = sub;
  }

  @Override
    public void initialize() {
      armSubsystem.resetPivotingEncoder();
      armSubsystem.resetTelescopingEncoder();
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