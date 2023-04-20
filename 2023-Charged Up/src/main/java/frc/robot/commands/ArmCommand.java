package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ArmSubsystem;


public class ArmCommand extends CommandBase {

  private double angleSetpoint;
  private ArmSubsystem armSubsystem;
  private SlewRateLimiter angleRateLimiter;
  private double rawLeftJoystickInput;
  private double leftJoystickInput;
  private boolean manual_override;
  private int a_press_counter;
  private boolean manual_tripped;

  private double lengthSetpoint;
  private SlewRateLimiter lengthRateLimiter;
  private double rawRightJoystickInput;
  private double rightJoystickInput;


  public ArmCommand (ArmSubsystem sub) {
    addRequirements(sub);
    armSubsystem = sub;
    
    //limits the change of a given variable to never exceed Constants.armSlewRate per second
    angleRateLimiter = new SlewRateLimiter(Constants.angleSlewRate);
    lengthRateLimiter = new SlewRateLimiter(Constants.lengthSlewRate);

    manual_override = false;
    a_press_counter = 0;
    manual_tripped = false;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //sets the setpoint to the current location of the arm on startup so that the arm doesn't move
    zeroArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    // There are three sections to this execute section. 
    // The big if else statement will periodically change the setpoint based on the OI input.
    // The clamp line will make sure we don't exceed our limits
    // The setpoint line will set the angle based on setpoint.

    rawLeftJoystickInput = OI.getOperator().getLeftY();
    rawRightJoystickInput = OI.getOperator().getRightY();

    // Set setPoint values with buttons. only one at a time
    if ( OI.getOperator().start().getAsBoolean() ) { //Manual Overide
      // angleSetpoint = Constants.ARM_ANGLE_LOW;
      // lengthSetpoint = Constants.ARM_LENGTH_GROUND;
      if (!manual_tripped) {
        if (a_press_counter >= 40) {
          manual_override = !manual_override;
          armSubsystem.stop();
          a_press_counter = 0;
          manual_tripped = true;
          OI.getXOperator().setRumble(RumbleType.kBothRumble, 0.5);
        } else {
          a_press_counter++;
        }
      }
      
    } else if ( OI.getOperator().a().getAsBoolean() ) { //low position
      angleSetpoint = Constants.ARM_ANGLE_LOW;
      lengthSetpoint = Constants.ARM_LENGTH_MIN;
    }  else if ( OI.getOperator().b().getAsBoolean() ) { //medium position
      angleSetpoint = Constants.ARM_ANGLE_MID;
      lengthSetpoint = Constants.ARM_LENGTH_MID;
    } else if ( OI.getOperator().y().getAsBoolean() ) { // high position
      angleSetpoint = Constants.ARM_ANGLE_HIGH;
      lengthSetpoint = Constants.ARM_LENGTH_MAX;
    } else if ( OI.getOperator().x().getAsBoolean() ) { // neutral position
      angleSetpoint = Constants.ARM_ANGLE_NEUTRAL;
      lengthSetpoint = Constants.ARM_LENGTH_MIN;
    } else {
      a_press_counter = 0;
      manual_tripped = false;
      OI.getXOperator().setRumble(RumbleType.kBothRumble, 0.0);
    }

    //manual set angle
    if ( Math.abs( rawLeftJoystickInput ) > Constants.controllerDeadzone ) {
      if (manual_override) {
        armSubsystem.setArmUnlocked();
        armSubsystem.setPivotSpeed(rawLeftJoystickInput);
      } else {
        leftJoystickInput = rawLeftJoystickInput;
        angleSetpoint = angleRateLimiter.calculate(angleSetpoint + leftJoystickInput);     
      }
    } else {
      if (manual_override) {
        armSubsystem.setPivotSpeed(0.0);
        armSubsystem.setArmLocked();
      }
    }

    //manual set length
    if ( Math.abs( rawRightJoystickInput ) > Constants.controllerDeadzone) {
      // if (manual_override) {
      //   armSubsystem.setTelescopingSpeed(-rawRightJoystickInput);
      // } else {
        rightJoystickInput = -rawRightJoystickInput;
        lengthSetpoint = lengthRateLimiter.calculate(lengthSetpoint + 20000 * rightJoystickInput);
      //}
    } else {
      // if (manual_override) {
      //   armSubsystem.setTelescopingSpeed(0.0);
      // }
    }

    angleSetpoint = MathUtil.clamp(angleSetpoint, -Constants.pivotHardLimit, Constants.pivotHardLimit);
    if (!manual_override) {
      armSubsystem.setPivotAngle(angleSetpoint);
    }
    armSubsystem.setArmLength(lengthSetpoint);
    
    SmartDashboard.putBoolean("Arm Manual Override Mode", manual_override);
}

  private void zeroArm() {
    armSubsystem.resetTelescopingEncoder();
    lengthSetpoint = armSubsystem.getTelescopingEncoder();
    angleSetpoint = armSubsystem.getPivotAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
    //bikeBreakSubsystem.setArmLocked();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return pivotingArmSubsystem.PID.atSetpoint();
  }

     
}