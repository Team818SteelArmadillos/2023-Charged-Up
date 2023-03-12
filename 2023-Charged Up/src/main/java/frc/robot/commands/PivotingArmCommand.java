package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.PivotingArmSubsystem;


public class PivotingArmCommand extends CommandBase {

  private double setPoint;
  private PivotingArmSubsystem pivotingArmSubsystem;
  private DutyCycleEncoder encoder;
  private SlewRateLimiter rateLimit;
  private double joystickOutput;
  private double rawJoystickOutput;

  public PivotingArmCommand (PivotingArmSubsystem sub) {
    addRequirements(sub);
    pivotingArmSubsystem = sub;
    
    //limits the change of a given variable to never exceed Constants.armSlewRate per second
    rateLimit = new SlewRateLimiter(Constants.armSlewRate);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //sets the setpoint to the current location of the arm on startup so that the arm doesn't move
    setPoint = pivotingArmSubsystem.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // There are three sections to this execute section. 
    // The big if else statement will periodically change the setpoint based on the OI input.
    // The clamp line will make sure we don't exceed our limits
    // The setpoint line will set the angle based on setpoint.

    rawJoystickOutput = OI.getOperator().getLeftY();

    // Set setPoint value
    if ( OI.getOperator().a().getAsBoolean() ) {
      setPoint = Constants.armAngles[0];
    } else if ( OI.getOperator().b().getAsBoolean() ) {
      setPoint = Constants.armAngles[1];
    } else if ( OI.getOperator().y().getAsBoolean() ) {
      setPoint = Constants.armAngles[2];
    } else if ( Math.abs( rawJoystickOutput ) > Constants.controllerDeadzone ) {
      joystickOutput = rawJoystickOutput;
      setPoint = rateLimit.calculate(setPoint + joystickOutput);
    } else {
      // do nothing
    }
  
    MathUtil.clamp(setPoint, Constants.pivotHardLimit, -Constants.pivotHardLimit);
    pivotingArmSubsystem.setPivotAngle(setPoint);
    SmartDashboard.putNumber("Arm Angle", pivotingArmSubsystem.getAngle());

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