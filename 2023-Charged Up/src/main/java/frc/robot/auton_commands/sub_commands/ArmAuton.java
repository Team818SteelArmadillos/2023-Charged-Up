// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmAuton extends CommandBase {
  ArmSubsystem m_armSubsystem;
  Timer timeout;
  
  double angleSetpoint;
  double lengthSetpoint;
  double last_lengthSetpoint;
  int state;

  /** Creates a new ArmAuton. */
  public ArmAuton(ArmSubsystem pivotingArmSubsystem, int State) {
    addRequirements(pivotingArmSubsystem);
     m_armSubsystem = pivotingArmSubsystem;
     state = State;
     last_lengthSetpoint = 0;

     timeout = new Timer();
  }

  @Override
  public void initialize() {
    timeout.reset();
    timeout.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {

    if (state == Constants.ARM_LOW_STATE) { //low position 
      angleSetpoint = Constants.ARM_ANGLE_LOW;
      lengthSetpoint = Constants.ARM_LENGTH_GROUND;
    } else if (state == Constants.ARM_MID_STATE) { //medium position
      angleSetpoint = Constants.ARM_ANGLE_MID;
      lengthSetpoint = Constants.ARM_LENGTH_MID;
    } else if (state == Constants.ARM_HIGH_STATE) { // high position
      angleSetpoint = Constants.ARM_ANGLE_HIGH;
      lengthSetpoint = Constants.ARM_LENGTH_MAX;
    } else if (state == Constants.ARM_NEUTRAL_STATE) { // neutral position
      angleSetpoint = Constants.ARM_ANGLE_NEUTRAL;
      lengthSetpoint = Constants.ARM_LENGTH_MIN;
    } else {
      //do nothing
    }
    
    angleSetpoint = MathUtil.clamp(angleSetpoint, -Constants.pivotHardLimit, Constants.pivotHardLimit);
    m_armSubsystem.setPivotAngle(angleSetpoint);
    m_armSubsystem.setArmLength(lengthSetpoint);
    
    // if (m_armSubsystem.PivotingPID.atSetpoint() || lengthSetpoint < m_armSubsystem.getTelescopingEncoder()) {
    //   
    // }
  }
  
  @Override
  public void end(boolean interrupted) {
      timeout.stop();
      m_armSubsystem.setArmLocked();
  }

  @Override
  public boolean isFinished() {
    return  m_armSubsystem.PivotingPID.atSetpoint() && m_armSubsystem.onSetPoint(lengthSetpoint)|| timeout.hasElapsed(2.5) ;
  }
}
