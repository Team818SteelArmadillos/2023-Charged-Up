package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.TelescopingArmSubsystem;
import frc.robot.subsystems.PivotingArmSubsystem;

public class EncoderCommand extends CommandBase {

  private PivotingArmSubsystem pivotingArmSubsystem;
  private TelescopingArmSubsystem telescopingArmSubsystem;

  public EncoderCommand(PivotingArmSubsystem sub, TelescopingArmSubsystem sub1) {
    addRequirements(sub, sub1);
    pivotingArmSubsystem = sub;
    telescopingArmSubsystem = sub1;
  }

  @Override
    public void initialize() {
      pivotingArmSubsystem.resetEncoder();
      telescopingArmSubsystem.resetEncoder();
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