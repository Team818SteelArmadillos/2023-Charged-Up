package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoBalanceCommand extends CommandBase {

  private SwerveDrivetrain swerveDrivetrain;
  private PivotingArmSubsystem pivotingArmSubsystem;

  public PIDController pid;

  public double pidOutput;
  public double pitch;
  public Translation2d translation;

  public SlewRateLimiter speedRateLimiter;
  
  public double balanceSpeed = 0.1;

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

      //create a translation 2d with the pid onnly affecting the y-axis
      if ( pitch > Constants.csTolerance ) {
        translation = new Translation2d(balanceSpeed, 0);
      } else if ( pitch < -Constants.csTolerance ) {
        translation = new Translation2d(-balanceSpeed, 0);
      } else {
        translation = new Translation2d(0, 0);
      }
      
      swerveDrivetrain.drive(translation, 0, true, true);
    
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return (pitch < Constants.csTolerance && pitch > -Constants.csTolerance);
  }
}