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
import frc.robot.subsystems.PistonClawSubsystem;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
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
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  private final PivotingArmSubsystem m_pivotingArmSubsystem = new PivotingArmSubsystem();
  private final TelescopingArmSubsystem m_telescopingArmSubsystem = new TelescopingArmSubsystem();

  //TelescopingArmCommand instances for different lengths
  public final TelescopingArmCommand m_TelescopingArmZeroCommand = new TelescopingArmCommand(0);
  public final TelescopingArmCommand m_TelescopingArmLowCommand = new TelescopingArmCommand(1);
  public final TelescopingArmCommand m_TelescopingArmMediumCommand = new TelescopingArmCommand(2);
  public final TelescopingArmCommand m_TelescopingGrabHighCommand = new TelescopingArmCommand(3);
  public final TelescopingArmCommand m_TelescopingArmHighCommand = new TelescopingArmCommand(4);
  
  //PivotingArmCommand instances for different arm angles
  public final PivotingArmCommand m_PivotingArmGroundCommand = new PivotingArmCommand(0, m_pivotingArmSubsystem); // Sets angle to 0 deg
  public final PivotingArmCommand m_PivotingArmMediumCommand = new PivotingArmCommand(1, m_pivotingArmSubsystem); // sets angle to 30 deg
  public final PivotingArmCommand m_PivotingArmHighCommand = new PivotingArmCommand(2, m_pivotingArmSubsystem); // sets angle to 45 dm_eg
  public final PivotingArmCommand m_PivotingGrabHighCommand = new PivotingArmCommand(3, m_pivotingArmSubsystem); // sets angle to 45 dm_eg
  public final PivotingArmCommand m_PivotingArmRestingCommand = new PivotingArmCommand(4, m_pivotingArmSubsystem); // Sets angle to 90 deg
  
  //reversed pivoting arm commands
  public final PivotingArmCommand r_PivotingArmGroundCommand = new PivotingArmCommand(5, m_pivotingArmSubsystem); // Sets angle to 0 deg
  public final PivotingArmCommand r_PivotingArmMediumCommand = new PivotingArmCommand(6, m_pivotingArmSubsystem); // sets angle to 30 deg
  public final PivotingArmCommand r_PivotingArmHighCommand = new PivotingArmCommand(7, m_pivotingArmSubsystem); // sets angle to 45 dm_eg
  public final PivotingArmCommand r_PivotingGrabHighCommand = new PivotingArmCommand(8, m_pivotingArmSubsystem); // sets angle to 45 dm_eg
  public final PivotingArmCommand r_PivotingArmRestingCommand = new PivotingArmCommand(9, m_pivotingArmSubsystem); // Sets angle to 90 deg
  
  //public Trigger operatorX = new Trigger(OI.getOperator().x());
  
  

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
    OI.getOperator().L1().onTrue( new ClawCommand() );
    OI.getOperator().touchpad().onTrue( new BikeBreakCommand() );
    OI.getOperator().R2().whileTrue( new ClawWheelCommand(0) );
    OI.getOperator().L2().whileTrue( new ClawWheelCommand(1) );
    

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