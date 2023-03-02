package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.BikeBreakSubsystem;
import frc.robot.subsystems.PivotingArmSubsystem;

public class BikeBreakCommand extends CommandBase {
    
  public static int _state;

  public BikeBreakCommand (BikeBreakSubsystem sub) {
    addRequirements(sub);
  }

  @Override
    public void initialize() {
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if ( OI.getOperator().getXButtonPressed() ) { BikeBreakSubsystem.setArmUnlocked(); } // X
      if ( OI.getOperator().getYButtonPressed() ) { BikeBreakSubsystem.setArmLocked(); } // Y
      if(OI.getOperator().getAButtonPressed()){
        PivotingArmSubsystem.setPivotSpeed(OI.getOperator().getLeftY());
      }
      if(OI.getOperator().getAButtonReleased()){
        PivotingArmSubsystem.setPivotSpeed(0);
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