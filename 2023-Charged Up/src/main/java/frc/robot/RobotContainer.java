// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ArmSubsystem;
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
  //private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kX.value);


  /* Subsystems */
  private final ClawSubsystem m_ClawSubsystem = new ClawSubsystem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  //private final LimeNetwork m_LimeNetwork = new LimeNetwork();
  //private final Pathplanning m_Pathplanning = new Pathplanning();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  
  //pivoting manual command
  public final ArmCommand m_ArmCommand = new ArmCommand(m_armSubsystem);
  public final AutoBalanceCommand m_autoBalanceCommand = new AutoBalanceCommand(m_swerveDrivetrain, m_LedSubsystem);

  //private final DriveDistance m_driveDistance = new DriveDistance(m_swerveDrivetrain, new Pose2d(3.0, 0.0, new Rotation2d(0.0)), fieldRelative, openLoop);

  /*
  public final PivotingArmCommand m_manualPivotingArmCommand = new PivotingArmCommand(-1, m_pivotingArmSubsystem, m_BikeBreakSubsystem);
  
  //PivotingArmCommand instances for different arm angles
  public final PivotingArmCommand m_PivotingArmGroundCommand = new PivotingArmCommand(0, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // Sets angle to 0 deg
  public final PivotingArmCommand m_PivotingArmMediumCommand = new PivotingArmCommand(1, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // sets angle to 30 deg
  public final PivotingArmCommand m_PivotingArmHighCommand = new PivotingArmCommand(2, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // sets angle to 45 dm_eg
  public final PivotingArmCommand m_PivotingGrabHighCommand = new PivotingArmCommand(3, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // sets angle to 45 dm_eg
  public final PivotingArmCommand m_PivotingArmRestingCommand = new PivotingArmCommand(4, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // Sets angle to 90 deg
  
  //reversed pivoting arm commands
  public final PivotingArmCommand r_PivotingArmGroundCommand = new PivotingArmCommand(5, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // Sets angle to 0 deg
  public final PivotingArmCommand r_PivotingArmMediumCommand = new PivotingArmCommand(6, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // sets angle to 30 deg
  public final PivotingArmCommand r_PivotingArmHighCommand = new PivotingArmCommand(7, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // sets angle to 45 dm_eg
  public final PivotingArmCommand r_PivotingGrabHighCommand = new PivotingArmCommand(8, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // sets angle to 45 dm_eg
  public final PivotingArmCommand r_PivotingArmRestingCommand = new PivotingArmCommand(9, m_pivotingArmSubsystem, m_BikeBreakSubsystem); // Sets angle to 90 deg
  */

  //claw command
  public final ClawCommand m_ClawCommand = new ClawCommand(m_ClawSubsystem, m_LedSubsystem);

  //bikebreak command

  //claw wheel commands
  public final ClawWheelCommand m_IntakeIn = new ClawWheelCommand(0, m_ClawSubsystem);
  public final ClawWheelCommand m_IntakeOut = new ClawWheelCommand(1, m_ClawSubsystem);

 
 //led command
  //public final LEDCommand m_LEDColorCommand = new LEDCommand(m_LedSubsystem);

  //reset encoder command
  public final EncoderCommand m_EncoderCommand = new EncoderCommand(m_armSubsystem);

  // auton commands
  private final BlueMiddleAuton m_BlueMiddleAuton = new BlueMiddleAuton(m_armSubsystem, m_swerveDrivetrain, m_ClawSubsystem, m_LedSubsystem);
  private final BlueRightAuton m_BlueRightAuton = new BlueRightAuton(m_armSubsystem, m_swerveDrivetrain, m_ClawSubsystem, m_LedSubsystem);
  private final BlueMiddleAuton m_BlueBalanceAuton = new BlueMiddleAuton(m_armSubsystem, m_swerveDrivetrain, m_ClawSubsystem, m_LedSubsystem);
  // auton chooser
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  public RobotContainer() {

    /* Set Drive as default command*/
    boolean fieldRelative = true;
    boolean openLoop = true;
    
    m_armSubsystem.setDefaultCommand(m_ArmCommand);
    m_swerveDrivetrain.setDefaultCommand(new SwerveDrive(m_swerveDrivetrain, 
      m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));

    //m_zeroGyro.onTrue(new InstantCommand(() -> m_swerveDrivetrain.resetGyro()));
    
    // Initializie auton chooser in smartdashboard
    m_autoChooser.setDefaultOption("Blue Middle Auton", m_BlueMiddleAuton);
    m_autoChooser.addOption("Blue Right Auton", m_BlueRightAuton);
    m_autoChooser.addOption("Blue Balance Auton", m_BlueBalanceAuton);
    SmartDashboard.putData("Auton Choices", m_autoChooser);

    //m_swerveDrivetrain.zeroModules();
    /* Initialize diagnostics subystem */
    //m_diagnostics = new Diagnostics(m_swerveDrivetrain, m_climber, m_intake, m_feeder, m_shooter, m_actuator);
    
    /* Configure the button bindings */
    configureButtonBindings();
  }

  public void configureButtonBindings() {

    OI.getOperator().leftBumper().whileTrue( m_ClawCommand );
    OI.getOperator().rightTrigger().whileTrue( m_IntakeOut );
    OI.getOperator().leftTrigger().whileTrue(m_IntakeIn );
    OI.getDriver().y().whileTrue( m_autoBalanceCommand );
    //OI.getOperator().povUp().whileTrue( m_BikeBreakCommand );    

  }

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


    return m_autoChooser.getSelected();//m_BlueMiddleAuton;

  }
  
}