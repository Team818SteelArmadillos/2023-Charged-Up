// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.subsystems.TelescopingArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmAuton extends CommandBase {
  PivotingArmSubsystem m_PivotingArmSubsystem;
  TelescopingArmSubsystem m_TelescopingArmSubsystem;
  
  double angleSetpoint;
  double lengthSetpoint;
  int state;

  /** Creates a new ArmAuton. */
  public ArmAuton(PivotingArmSubsystem pivotingArmSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, int State) {
    addRequirements(pivotingArmSubsystem, telescopingArmSubsystem);
     m_PivotingArmSubsystem = pivotingArmSubsystem;
     m_TelescopingArmSubsystem = telescopingArmSubsystem;
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
    m_PivotingArmSubsystem.setPivotAngle(angleSetpoint);

    // Debug MAnual speed control
    //telescopingArmSubsystem.setSpeed(OI.getOperator().getRightY());
    lengthSetpoint = MathUtil.clamp(lengthSetpoint, -Constants.maximumArmLength, Constants.maximumArmLength);
    m_TelescopingArmSubsystem.setArmLength(lengthSetpoint);
  }
  @Override
  public void end(boolean interrupted) {
      m_PivotingArmSubsystem.setArmLocked();
  }

  @Override
  public boolean isFinished() {
    return  m_PivotingArmSubsystem.PID.atSetpoint() && m_TelescopingArmSubsystem.onSetPoint(lengthSetpoint);
  }
}
