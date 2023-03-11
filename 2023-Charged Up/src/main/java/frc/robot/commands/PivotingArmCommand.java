package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.Constants;
import frc.robot.OI;


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

    rateLimit = new SlewRateLimiter(Constants.armSlewRate);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // bikeBreakSubsystem.setArmUnlocked();
    setPoint = pivotingArmSubsystem.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rawJoystickOutput = OI.getOperator().getLeftY();

    // Set setPoint value
    if ( OI.getOperator().a().getAsBoolean() ) {
      setPoint = Constants.armAngles[0];
    } else if ( OI.getOperator().b().getAsBoolean() ) {
      setPoint = Constants.armAngles[1];
    } else if ( OI.getOperator().y().getAsBoolean() ) {
      setPoint = Constants.armAngles[2];
    } else if ( OI.getOperator().x().getAsBoolean() ) {
      setPoint = pivotingArmSubsystem.getAngle();
    } else {
      joystickOutput = ( Math.abs( rawJoystickOutput ) > 0.1 ) ? rawJoystickOutput : 0; 
      setPoint = rateLimit.calculate(setPoint + joystickOutput);
    }
      

    // Limit the setPoint to our min/max
    if (setPoint > Constants.pivotHardLimit) {
      setPoint = Constants.pivotHardLimit;
    } else if (setPoint < -Constants.pivotHardLimit) {
      setPoint = -Constants.pivotHardLimit;
    } else {
      // do nothing
    }

    // Run arm go to setPoint
    pivotingArmSubsystem.setPivotAngle(setPoint);

    
    SmartDashboard.putNumber("Arm Angle", pivotingArmSubsystem.getAngle());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //BikeBreakSubsystem.setArmLocked();
    
    pivotingArmSubsystem.setPivotSpeed(0);
    //bikeBreakSubsystem.setArmLocked();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return pivotingArmSubsystem.PID.atSetpoint();
  }

     
}