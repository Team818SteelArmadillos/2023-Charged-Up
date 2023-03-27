package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    if ( OI.getOperator().a().getAsBoolean() ) { //low position 
      angleSetpoint = Constants.armAngles[4];
      lengthSetpoint = Constants.armLengths[1];
    } else if ( OI.getOperator().b().getAsBoolean() ) { //medium position
      angleSetpoint = Constants.armAngles[3];
      lengthSetpoint = Constants.armLengths[2];
    } else if ( OI.getOperator().y().getAsBoolean() ) { // high position
      angleSetpoint = Constants.armAngles[2];
      lengthSetpoint = Constants.armLengths[3];
    } else if ( OI.getOperator().x().getAsBoolean() ) { // neutral position
      angleSetpoint = Constants.armAngles[0];
      lengthSetpoint = Constants.armLengths[0];
    } else {
      //do nothing
    }
    
    //manual set angle
    if ( Math.abs( rawLeftJoystickInput ) > Constants.controllerDeadzone ) {
      leftJoystickInput = rawLeftJoystickInput;
      angleSetpoint = angleRateLimiter.calculate(angleSetpoint + leftJoystickInput);
    }

    //manual set length
    if ( Math.abs( rawRightJoystickInput ) > Constants.controllerDeadzone ) {
      rightJoystickInput = -rawRightJoystickInput;
      lengthSetpoint = lengthRateLimiter.calculate(lengthSetpoint + 20000 * rightJoystickInput);
    }

    if ( armSubsystem.getLimitswitch() ) {
      armSubsystem.resetTelescopingEncoder();
    }

    SmartDashboard.putBoolean("limit switch", armSubsystem.getLimitswitch());
    angleSetpoint = MathUtil.clamp(angleSetpoint, -Constants.pivotHardLimit, Constants.pivotHardLimit);
    armSubsystem.setPivotAngle(angleSetpoint);

    lengthSetpoint = MathUtil.clamp(lengthSetpoint, -Constants.maximumArmLength, Constants.maximumArmLength);
    if (angleSetpoint == Constants.armAngles[0]) {
      armSubsystem.setArmLength(lengthSetpoint);
    } else if (armSubsystem.onPivotingSetPoint() && armSubsystem.isBikeBreakEngaged()) {
        armSubsystem.setArmLength(lengthSetpoint);
    }
    
    
    SmartDashboard.putNumber("Telescoping Encoder", armSubsystem.getTelescopingEncoder());
    SmartDashboard.putNumber("Telescoping Setpoint", lengthSetpoint);
    
    SmartDashboard.putNumber("setpoint angle", angleSetpoint);

    
    //old debug features
  }

  private void zeroArm() {
    armSubsystem.resetTelescopingEncoder();
    lengthSetpoint = armSubsystem.getTelescopingEncoder();
    angleSetpoint = armSubsystem.getPivotAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //BikeBreakSubsystem.setArmLocked();
    //bikeBreakSubsystem.setArmLocked();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return pivotingArmSubsystem.PID.atSetpoint();
  }

     
}