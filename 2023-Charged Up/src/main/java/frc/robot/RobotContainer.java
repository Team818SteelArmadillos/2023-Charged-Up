// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;

import frc.robot.automodes.Auto;
import frc.robot.auton_commands.sub_commands.AutonResetOdometry;
import frc.robot.auton_commands.sub_commands.IntakeInAuton;
import frc.robot.auton_commands.sub_commands.ResetOdometry;

 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  /* Subsystems */
  public static final CTRSwerveSubsystem m_swerveSubsystem = new CTRSwerveSubsystem();
  public static final ClawSubsystem m_ClawSubsystem = new ClawSubsystem();
  public static final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final Vision m_visionSubsystem = new Vision();


  //Test Auton
  
  //arm command
  public final ArmCommand m_ArmCommand = new ArmCommand(m_armSubsystem);


  public final OdometryMonitor m_odometrymonitor = new OdometryMonitor(m_visionSubsystem, m_swerveSubsystem);
  //claw commands
  public final ClawModeToggleCommand m_ClawCommand = new ClawModeToggleCommand(m_ClawSubsystem);
  public final ClawWheelCommand m_IntakeIn = new ClawWheelCommand(0, m_ClawSubsystem);
  public final ClawWheelCommand m_IntakeOut = new ClawWheelCommand(1, m_ClawSubsystem);

  //Auton Instant Commands
  public final AutonResetOdometry m_AutonResetOdometry = new AutonResetOdometry(m_visionSubsystem, m_swerveSubsystem);
  //server command
  public final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_swerveSubsystem);

  //intake command
  public final IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);

  public final ResetOdometry m_ResetOdometry = new ResetOdometry(m_visionSubsystem, m_swerveSubsystem);
  // auton chooser
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private final Auto m_Auto = new Auto();

  public RobotContainer() {

    m_swerveSubsystem.setDefaultCommand(m_swerveDriveCommand);
    m_armSubsystem.setDefaultCommand(m_ArmCommand);
    m_intakeSubsystem.setDefaultCommand(m_intakeCommand);
    m_visionSubsystem.setDefaultCommand(m_odometrymonitor);
    


    //onePointFiveBalance m_onePointFiveBalance = new onePointFiveBalance;
    // Initializie auton chooser in smartdashboard
    m_autoChooser.setDefaultOption("Mid 1.5 Balance", m_Auto.onePointFiveBalanceMid());
    m_autoChooser.addOption("Mid Cross Balance", m_Auto.mobilityBalance());
    m_autoChooser.addOption("Clear High 3", m_Auto.threePieceOpenHigh());
    m_autoChooser.addOption("Clear Low 3", m_Auto.threePieceOpenLow());
    m_autoChooser.addOption("Bump Low 3", m_Auto.threePieceBumpLow());
    m_autoChooser.addOption("Blue Clear 2.5 Balance", m_Auto.twoPointFiveBalanceOpen());
    SmartDashboard.putData("Auton Choices", m_autoChooser);
    
    /* Configure the button bindings */
    configureButtonBindings();
  }

  public void configureButtonBindings() {
    OI.getOperator().leftBumper().whileTrue(m_ClawCommand);
    OI.getOperator().rightTrigger().whileTrue(m_IntakeOut);
    OI.getOperator().leftTrigger().whileTrue(m_IntakeIn);
    
    OI.getOperator().leftStick().whileTrue(m_ResetOdometry);
  }

  //Commands for Auton
  public void resetOdometry(){
    m_ResetOdometry.schedule();
  }

  public void intakeIn(){
    m_IntakeIn.schedule();
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