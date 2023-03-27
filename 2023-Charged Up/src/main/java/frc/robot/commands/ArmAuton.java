// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmAuton extends CommandBase {
  ArmSubsystem m_armSubsystem;
  
  double angleSetpoint;
  double lengthSetpoint;
  int state;

  /** Creates a new ArmAuton. */
  public ArmAuton(ArmSubsystem pivotingArmSubsystem, int State) {
    addRequirements(pivotingArmSubsystem);
     m_armSubsystem = pivotingArmSubsystem;
     state = State;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {

    if (state == 0) { //low position 
      angleSetpoint = Constants.armAngles[3];
      lengthSetpoint = Constants.armLengths[1];
    } else if (state == 1) { //medium position
      angleSetpoint = Constants.armAngles[3];
      lengthSetpoint = Constants.armLengths[2];
    } else if (state == 2) { // high position
      angleSetpoint = Constants.armAngles[2];
      lengthSetpoint = Constants.armLengths[3];
    } else if (state == 3) { // neutral position
      angleSetpoint = Constants.armAngles[0];
      lengthSetpoint = Constants.armLengths[0];
    } else {
      //do nothing
    }
    
    angleSetpoint = MathUtil.clamp(angleSetpoint, -Constants.pivotHardLimit, Constants.pivotHardLimit);
    m_armSubsystem.setPivotAngle(angleSetpoint);

    // Debug MAnual speed control
    //telescopingArmSubsystem.setSpeed(OI.getOperator().getRightY());
    lengthSetpoint = MathUtil.clamp(lengthSetpoint, -Constants.maximumArmLength, Constants.maximumArmLength);
    if (state == 3) {
      m_armSubsystem.setArmLength(lengthSetpoint);
    } else if (m_armSubsystem.onPivotingSetPoint() && m_armSubsystem.isBikeBreakEngaged()) {
      m_armSubsystem.setArmLength(lengthSetpoint);
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
