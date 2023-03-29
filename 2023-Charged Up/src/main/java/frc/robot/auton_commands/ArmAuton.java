// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmAuton extends CommandBase {
  ArmSubsystem m_armSubsystem;
  
  double angleSetpoint;
  double lengthSetpoint;
  int state;
  int break_engaged_counter;

  /** Creates a new ArmAuton. */
  public ArmAuton(ArmSubsystem pivotingArmSubsystem, int State) {
    addRequirements(pivotingArmSubsystem);
     m_armSubsystem = pivotingArmSubsystem;
     state = State;

     break_engaged_counter = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {

    if (state == Constants.ARM_LOW_STATE) { //low position 
      angleSetpoint = Constants.ARM_ANGLE_LOW;
      lengthSetpoint = Constants.ARM_LENGTH_MIN;
    } else if (state == Constants.ARM_MID_STATE) { //medium position
      angleSetpoint = Constants.ARM_ANGLE_MID;
      lengthSetpoint = Constants.ARM_LENGTH_MID;
    } else if (state == Constants.ARM_ANGLE_HIGH) { // high position
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

    if (m_armSubsystem.isBikeBreakEngaged()) {
      break_engaged_counter++;
    } else {
      break_engaged_counter = 0;
    }
    
    if (break_engaged_counter >= 10 || state == Constants.ARM_NEUTRAL_STATE) {
      m_armSubsystem.setArmLength(lengthSetpoint);
      break_engaged_counter = 10; // prevent overflow
    }
  }
  
  @Override
  public void end(boolean interrupted) {
      m_armSubsystem.setArmLocked();
  }

  @Override
  public boolean isFinished() {
    return  m_armSubsystem.PivotingPID.atSetpoint() && m_armSubsystem.onSetPoint(lengthSetpoint);
  }
}
