// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PivotingArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmAuton extends CommandBase {
  PivotingArmSubsystem m_PivotingArmSubsystem;
  /** Creates a new ArmAuton. */
  public ArmAuton(PivotingArmSubsystem pivotingArmSubsystem) {
    addRequirements(pivotingArmSubsystem());
     m_PivotingArmSubsystem = pivotingArmSubsystem;

    if (a ) { //low position 
      angleSetpoint = Constants.armAngles[3];
      lengthSetpoint = Constants.armLengths[1];
    } else if ( b ) { //medium position
      angleSetpoint = Constants.armAngles[2];
      lengthSetpoint = Constants.armLengths[2];
    } else if ( y) { // high position
      angleSetpoint = Constants.armAngles[2];
      lengthSetpoint = Constants.armLengths[3];
    } else if ( x ) { // pick up from feeder station
      zeroArm();
      // angleSetpoint = Constants.armAngles[1];
      // lengthSetpoint = Constants.armLengths[1];
    } else if ( bumper ) { // neutral position
      angleSetpoint = Constants.armAngles[0];
      lengthSetpoint = Constants.armLengths[0];
    } else {
      //do nothing
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleSetpoint = MathUtil.clamp(angleSetpoint, -Constants.pivotHardLimit, Constants.pivotHardLimit);
    m_pivotingArmSubsystem.setPivotAngle(angleSetpoint);

    // Debug MAnual speed control
    //telescopingArmSubsystem.setSpeed(OI.getOperator().getRightY());
    m_telescopingArmSubsystem.setArmLength(lengthSetpoint);
    lengthSetpoint = MathUtil.clamp(lengthSetpoint, -Constants.maximumArmLength, Constants.maximumArmLength);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
