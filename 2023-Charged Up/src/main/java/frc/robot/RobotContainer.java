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
import frc.robot.subsystems.LimeNetwork;
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
  private final LimeNetwork m_LimeNetwork = new LimeNetwork();
  //private final Pathplanning m_Pathplanning = new Pathplanning();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  
  //pivoting manual command
  public final ArmCommand m_ArmCommand = new ArmCommand(m_armSubsystem);
  public final AutoBalanceCommand m_autoBalanceCommand = new AutoBalanceCommand(m_swerveDrivetrain, m_LedSubsystem);

  //claw command
  public final ClawCommand m_ClawCommand = new ClawCommand(m_ClawSubsystem, m_LedSubsystem);

  //lime command
  public final LimelightCommand m_LimelightCommand = new LimelightCommand(m_LimeNetwork);

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
    m_LimeNetwork.setDefaultCommand(m_LimelightCommand);
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