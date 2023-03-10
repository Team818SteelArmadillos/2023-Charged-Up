package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.BikeBreakSubsystem;


public class PivotingArmCommand extends CommandBase {

  public static int _state;
  public static int[] revArmAngles;
  private PivotingArmSubsystem pivotingArmSubsystem;

  public PivotingArmCommand (int state, PivotingArmSubsystem sub) {
    addRequirements(sub);
    _state = state;
    pivotingArmSubsystem = sub;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //BikeBreakSubsystem.setArmLocked();
    
    //creates a reversed arm angles list from constants
    for (int i = 0; i < Constants.armAngles.length; i++) {
      revArmAngles[i] = -Constants.armAngles[i];
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (_state) {
      //forward angles
      case 0:  pivotingArmSubsystem.setPivotAngle(Constants.armAngles[0]); break;
      case 1:  pivotingArmSubsystem.setPivotAngle(Constants.armAngles[1]); break;
      case 2:  pivotingArmSubsystem.setPivotAngle(Constants.armAngles[2]); break;
      case 3:  pivotingArmSubsystem.setPivotAngle(Constants.armAngles[3]); break;
      case 4:  pivotingArmSubsystem.setPivotAngle(Constants.armAngles[4]); break;

      //reversed angles
      case 5:  pivotingArmSubsystem.setPivotAngle(revArmAngles[0]); break;
      case 6:  pivotingArmSubsystem.setPivotAngle(revArmAngles[1]); break;
      case 7:  pivotingArmSubsystem.setPivotAngle(revArmAngles[2]); break;
      case 8:  pivotingArmSubsystem.setPivotAngle(revArmAngles[3]); break;
      case 9:  pivotingArmSubsystem.setPivotAngle(revArmAngles[4]); break;
      default: break;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //BikeBreakSubsystem.setArmLocked();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotingArmSubsystem.PID.atSetpoint();
  }
     
}