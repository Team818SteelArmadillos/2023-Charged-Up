package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.BikeBreakSubsystem;
import frc.robot.subsystems.PivotingArmSubsystem;

public class BikeBreakCommand extends CommandBase {

  public BikeBreakCommand () {
  }

  @Override
    public void initialize() {
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (BikeBreakSubsystem.isOpen()){
        BikeBreakSubsystem.setArmLocked();
      } else {
        BikeBreakSubsystem.setArmUnlocked();
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