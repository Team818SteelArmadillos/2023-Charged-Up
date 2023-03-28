// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.CTRSwerve.SwerveDriveConstantsCreator;
import frc.robot.CTRSwerve.SwerveDriveTrainConstants;
import frc.robot.CTRSwerve.SwerveModuleConstants;
import frc.robot.commands.*;

import com.ctre.phoenixpro.configs.Slot0Configs;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  /* Subsystems */
  private final CTRSwerveSubsystem m_swerveSubsystem = new CTRSwerveSubsystem();
  private final ClawSubsystem m_ClawSubsystem = new ClawSubsystem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  //private final LimeNetwork m_LimeNetwork = new LimeNetwork();
  //private final Pathplanning m_Pathplanning = new Pathplanning();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  
  //arm command
  public final ArmCommand m_ArmCommand = new ArmCommand(m_armSubsystem);

  //claw commands
  public final ClawModeToggleCommand m_ClawCommand = new ClawModeToggleCommand(m_ClawSubsystem, m_LedSubsystem);
  public final ClawWheelCommand m_IntakeIn = new ClawWheelCommand(0, m_ClawSubsystem);
  public final ClawWheelCommand m_IntakeOut = new ClawWheelCommand(1, m_ClawSubsystem);

  //server command
  public final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_swerveSubsystem);

  // auton chooser
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(m_swerveDriveCommand);
    m_armSubsystem.setDefaultCommand(m_ArmCommand);
    
    // Initializie auton chooser in smartdashboard
    // m_autoChooser.setDefaultOption("Blue Middle Auton", m_BlueMiddleAuton);
    // m_autoChooser.addOption("Blue Right Auton", m_BlueRightAuton);
    // m_autoChooser.addOption("Blue Balance Auton", m_BlueBalanceAuton);
    SmartDashboard.putData("Auton Choices", m_autoChooser);
    
    /* Configure the button bindings */
    configureButtonBindings();
  }

  public void configureButtonBindings() {
    OI.getOperator().leftBumper().whileTrue(m_ClawCommand);
    OI.getOperator().rightTrigger().whileTrue(m_IntakeOut);
    OI.getOperator().leftTrigger().whileTrue(m_IntakeIn);
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
    return m_autoChooser.getSelected();
  }
  
}