package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;


public class AutoBalanceCommand extends CommandBase {

  private SwerveDrivetrain swerveDrivetrain;
  LEDSubsystem led;

  public double roll;
  public Translation2d translation;

  public double balanceSpeed = 0.5;

  public AutoBalanceCommand (SwerveDrivetrain sub, LEDSubsystem sub1) {
    
    addRequirements(sub, sub1);
    swerveDrivetrain = sub;
    led = sub1;
    
  }

  @Override
    public void initialize() {
      led.setLEDRGB(249, 0, 79); //red
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


      roll = swerveDrivetrain.getRoll();

      //create a translation 2d with the pid onnly affecting the y-axis
      if ( roll > Constants.csTolerance ) {
        translation = new Translation2d(-balanceSpeed, 0);
      } else if ( roll < -Constants.csTolerance ) {
        translation = new Translation2d(balanceSpeed, 0);
      } else {
        translation = new Translation2d(0, 0);
      }
      
      swerveDrivetrain.drive(translation, 0, true, true);
    
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      translation = new Translation2d(0, 0);
      swerveDrivetrain.drive(translation, 0, true, true);
      led.setLEDRGB(0, 255, 64); //green
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return ( -Constants.csTolerance < roll && roll < Constants.csTolerance);
  }
}