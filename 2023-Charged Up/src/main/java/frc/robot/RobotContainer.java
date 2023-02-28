// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.SwerveDrivetrain;
 

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
  public final ClawCommand m_ClawCommand = new ClawCommand();

  //ClawWheelCommand instances
  public final ClawWheelCommand m_ClawWheelForwardCommand = new ClawWheelCommand(0);
  public final ClawWheelCommand m_ClawWheelReverseCommand = new ClawWheelCommand(1);

  //TelescopingArmCommand instances for different lengths
  public final TelescopingArmCommand m_TelescopingArmZeroCommand = new TelescopingArmCommand(0);
  public final TelescopingArmCommand m_TelescopingArmLowCommand = new TelescopingArmCommand(1);
  public final TelescopingArmCommand m_TelescopingArmMediumCommand = new TelescopingArmCommand(2);
  public final TelescopingArmCommand m_TelescopingGrabHighCommand = new TelescopingArmCommand(3);
  public final TelescopingArmCommand m_TelescopingArmHighCommand = new TelescopingArmCommand(4);
  
  //PivotingArmCommand instances for different arm angles
  public final PivotingArmCommand m_PivotingArmGroundCommand = new PivotingArmCommand(0); // Sets angle to 0 deg
  public final PivotingArmCommand m_PivotingArmMediumCommand = new PivotingArmCommand(1); // sets angle to 30 deg
  public final PivotingArmCommand m_PivotingArmHighCommand = new PivotingArmCommand(2); // sets angle to 45 dm_eg
  public final PivotingArmCommand m_PivotingGrabHighCommand = new PivotingArmCommand(3); // sets angle to 45 dm_eg
  public final PivotingArmCommand m_PivotingArmRestingCommand = new PivotingArmCommand(4); // Sets angle to 90 deg
  
  public RobotContainer() {

    /* Set Drive as default command*/
    boolean fieldRelative = true;
    boolean openLoop = true;
    m_swerveDrivetrain.setDefaultCommand(new SwerveDrive(m_swerveDrivetrain, 
      m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));
    
      //m_swerveDrivetrain.zeroModules();
    /* Initialize diagnostics subystem */
    //m_diagnostics = new Diagnostics(m_swerveDrivetrain, m_climber, m_intake, m_feeder, m_shooter, m_actuator);
    
    /* Configure the button bindings */
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
    m_zeroGyro.onTrue(new InstantCommand(() -> m_swerveDrivetrain.resetGyro()));

    // Bumpers (R1, R2, L1, L2)
    if ( OI.getOperator().getRightBumper() ) { m_ClawCommand.schedule(); } // R1
    
    if ( OI.getOperator().getRightTriggerAxis() >= 0.01) { m_ClawWheelForwardCommand.schedule(); } // R2

    if ( OI.getOperator().getLeftBumper() ) { Commands.parallel(m_PivotingArmRestingCommand, m_TelescopingArmZeroCommand); } // L1
  
    if ( OI.getOperator().getLeftTriggerAxis() >= 0.01 ) { m_ClawWheelReverseCommand.schedule(); } // L2
    
    // A, B, X, Y Buttons
    if ( OI.getOperator().getAButtonPressed() ) { Commands.parallel(m_PivotingArmGroundCommand, m_TelescopingArmLowCommand); } // A

    if ( OI.getOperator().getBButtonPressed() ) { Commands.parallel(m_PivotingArmMediumCommand, m_TelescopingArmMediumCommand); } // B
    
    if ( OI.getOperator().getXButtonPressed() ) { Commands.parallel(m_PivotingGrabHighCommand, m_TelescopingGrabHighCommand); } // X

    if ( OI.getOperator().getYButtonPressed() ) { Commands.parallel(m_PivotingArmHighCommand, m_TelescopingArmHighCommand); } // Y
    
    // DPAD ?
  }

  public Command getAutonInit() {
    return m_ClawCommand;
  }
  
  public Command getAutonPeriodic() {
    return m_ClawCommand;
  }

  private static SendableChooser<Command> autoChooser;

  /* Controllers */
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_PORT);

  /* Drive Axes */
  private final int m_translationAxis = XboxController.Axis.kLeftY.value;
  private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int m_rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kX.value);


  /* Subsystems */
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */


  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
  
}