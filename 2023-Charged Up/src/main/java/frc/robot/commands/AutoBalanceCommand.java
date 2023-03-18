package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoBalanceCommand extends CommandBase {

  private SwerveDrivetrain swerveDrivetrain;
  private PivotingArmSubsystem pivotingArmSubsystem;

  public AutoBalanceCommand (PivotingArmSubsystem sub, SwerveDrivetrain sub1) {
    addRequirements(sub);
    pivotingArmSubsystem = sub;
    swerveDrivetrain = sub1;
  }

  @Override
    public void initialize() {
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