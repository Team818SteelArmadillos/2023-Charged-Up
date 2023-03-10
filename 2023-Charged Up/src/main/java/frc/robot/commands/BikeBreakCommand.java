package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.BikeBreakSubsystem;
import frc.robot.subsystems.PivotingArmSubsystem;

public class BikeBreakCommand extends CommandBase {

  private BikeBreakSubsystem bikeBreakSubsystem;

  public BikeBreakCommand (BikeBreakSubsystem sub) {
    bikeBreakSubsystem = sub;
  }

  @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      bikeBreakSubsystem.toggle();
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