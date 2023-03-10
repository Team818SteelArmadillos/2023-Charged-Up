// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.BikeBreakSubsystem;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimeNetwork;
import frc.robot.subsystems.Pathplanning;
import frc.robot.subsystems.PistonClawSubsystem;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.TelescopingArmSubsystem;
 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  //private final DriveTrainCommand m_autoCommand = new DriveTrainCommand();
  
  //variables
  boolean facingForward;
  boolean fieldRelative;
  boolean openLoop;
  
  /* Controllers */
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_PORT);

  /* Drive Axes */
  private final int m_translationAxis = XboxController.Axis.kLeftY.value;
  private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int m_rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kX.value);


  /* Subsystems */
  private final BikeBreakSubsystem m_bikeBreakSubsystem = new BikeBreakSubsystem();
  private final ClawWheelsSubsystem m_clawWheelsSubsystem = new ClawWheelsSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final LimeNetwork m_limeNetwork = new LimeNetwork();
  private final Pathplanning m_pathplanning = new Pathplanning();
  private final PistonClawSubsystem m_PistonClawSubsystem = new PistonClawSubsystem();
  private final PivotingArmSubsystem m_pivotingArmSubsystem = new PivotingArmSubsystem();
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  private final TelescopingArmSubsystem m_telescopingArmSubsystem = new TelescopingArmSubsystem();  

  /*///
          Commands
  ///*/
  private final BikeBreakCommand m_BikeBreakCommand = new BikeBreakCommand(m_bikeBreakSubsystem);
  private final ClawCommand m_ClawCommand = new ClawCommand(m_PistonClawSubsystem);

  private final ClawWheelCommand m_ClawWheelForwardCommand = new ClawWheelCommand(m_clawWheelsSubsystem, 0);
  private final ClawWheelCommand m_ClawWheelReverseCommand = new ClawWheelCommand(m_clawWheelsSubsystem, 1);

  public RobotContainer() {

    /* Set Drive as default command*/
    boolean fieldRelative = true;
    boolean openLoop = true;
    
    m_swerveDrivetrain.setDefaultCommand(new SwerveDrive(m_swerveDrivetrain, 
      m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));

    m_zeroGyro.onTrue(new InstantCommand(() -> m_swerveDrivetrain.resetGyro()));
    
      //m_swerveDrivetrain.zeroModules();
    /* Initialize diagnostics subystem */
    //m_diagnostics = new Diagnostics(m_swerveDrivetrain, m_climber, m_intake, m_feeder, m_shooter, m_actuator);
    
    /* Configure the button bindings */
    configureButtonBindings();
  }

  public void configureButtonBindings() {

    //pivoting arm controls
    /*if (OI.getOperator().L1().getAsBoolean() == false) {
      OI.getOperator().cross().onTrue( Commands.parallel(m_PivotingArmGroundCommand, m_TelescopingArmLowCommand) );
      OI.getOperator().circle().onTrue( Commands.parallel(m_PivotingArmMediumCommand, m_TelescopingArmMediumCommand) );
      OI.getOperator().square().onTrue( Commands.parallel(m_PivotingGrabHighCommand, m_TelescopingGrabHighCommand) );
      OI.getOperator().triangle().onTrue( Commands.parallel(m_PivotingArmHighCommand, m_TelescopingArmHighCommand) );
    } else {
      OI.getOperator().cross().onTrue( Commands.parallel(r_PivotingArmGroundCommand, m_TelescopingArmLowCommand) );
      OI.getOperator().circle().onTrue( Commands.parallel(r_PivotingArmMediumCommand, m_TelescopingArmMediumCommand) );
      OI.getOperator().square().onTrue( Commands.parallel(r_PivotingGrabHighCommand, m_TelescopingGrabHighCommand) );
      OI.getOperator().triangle().onTrue( Commands.parallel(r_PivotingArmHighCommand, m_TelescopingArmHighCommand) ); 
    }
    */
    
    if ( Math.abs(OI.getOperator().getLeftY()) > 0.01 ) { m_pivotingArmSubsystem.setPivotSpeed(OI.getOperator().getLeftY()); }
    if ( Math.abs(OI.getOperator().getRightY()) > 0.01 ) { m_telescopingArmSubsystem.setSpeed(OI.getOperator().getRightY()); }
    OI.getOperator().L1().onTrue( m_ClawCommand );
    OI.getOperator().touchpad().whileTrue( m_BikeBreakCommand );
    OI.getOperator().R2().whileTrue( m_ClawWheelForwardCommand );
    OI.getOperator().L2().whileTrue( m_ClawWheelReverseCommand );
    

  }

  private static SendableChooser<Command> autoChooser;

  


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