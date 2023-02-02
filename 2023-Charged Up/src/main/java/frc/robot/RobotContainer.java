// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  //private final DriveTrainCommand m_autoCommand = new DriveTrainCommand();
  
  //ClawCommand instances
  private final ClawCommand m_ClawCommand = new ClawCommand();

  //ClawWheelCommand instances
  private final ClawWheelCommand m_ClawWheelForwardCommand = new ClawWheelCommand(0);
  private final ClawWheelCommand m_ClawWheelReverseCommand = new ClawWheelCommand(1);

  //TelescopingArmCommand instances for different lengths
  private final TelescopingArmCommand m_TelescopingArmZeroCommand = new TelescopingArmCommand(0);
  private final TelescopingArmCommand m_TelescopingArmLowCommand = new TelescopingArmCommand(1);
  private final TelescopingArmCommand m_TelescopingArmMediumCommand = new TelescopingArmCommand(2);
  private final TelescopingArmCommand m_TelescopingArmHighCommand = new TelescopingArmCommand(3);
  
  //PivotingArmCommand instances for different arm angles
  private final PivotingArmCommand m_PivotingArmGroundCommand = new PivotingArmCommand(0); // Sets angle to 15 deg
  private final PivotingArmCommand m_PivotingArmMediumCommand = new PivotingArmCommand(1); // sets angle to 45 deg
  private final PivotingArmCommand m_PivotingArmHighCommand = new PivotingArmCommand(2); // sets angle to 90 dm_eg
  private final PivotingArmCommand m_PivotingArmRestingCommand = new PivotingArmCommand(3); // Sets angle to 30 deg
  
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Bumpers (R1, R2, L1, L2)
    if ( OI.getOperator().getRightBumper() ) { m_ClawCommand.schedule(); } // R1
    
    if ( OI.getOperator().getRightTriggerAxis() >= 0.01) { m_ClawWheelForwardCommand.schedule(); } // R2

    if ( OI.getOperator().getLeftBumper() ) { Commands.parallel(m_PivotingArmRestingCommand, m_TelescopingArmZeroCommand); } // L1
  
    if ( OI.getOperator().getLeftTriggerAxis() >= 0.01 ) { m_ClawWheelReverseCommand.schedule(); } // L2
    
    // A, B, X, Y Buttons
    if ( OI.getOperator().getAButtonPressed() ) { Commands.parallel(m_PivotingArmGroundCommand, m_TelescopingArmLowCommand); } // A

    if ( OI.getOperator().getBButtonPressed() ) { Commands.parallel(m_PivotingArmMediumCommand, m_TelescopingArmMediumCommand); } // B
    
    if ( OI.getOperator().getXButtonPressed() ) {  } // X

    if ( OI.getOperator().getYButtonPressed() ) { Commands.parallel(m_PivotingArmHighCommand, m_TelescopingArmHighCommand); } // Y
    
    // DPAD ?
  }

}
