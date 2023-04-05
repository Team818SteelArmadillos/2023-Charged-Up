// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auton_commands.BClear_ScoreTwoAndBalanceAuton;
import frc.robot.auton_commands.BClear_ScoreTwoAndGrabAuton;
import frc.robot.auton_commands.BClear_ScoreThreeAuton;
import frc.robot.auton_commands.RClear_ScoreTwoAndGrabAuton;
import frc.robot.auton_commands.RWire_ScoreTwoCrossAuton;
import frc.robot.auton_commands.Mid_CrossGrabBalanceAuton;
import frc.robot.auton_commands.RClear_ScoreThreeAuton;
import frc.robot.auton_commands.RClear_ScoreTwoAndBalanceAuton;
import frc.robot.auton_commands.BWire_ScoreTwoCrossAuton;
import frc.robot.auton_commands.sub_commands.BalanceAuton;
import frc.robot.auton_commands.sub_commands.DriveToPositionAuton;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClawSubsystem;
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
  //private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  //private final LimeNetwork m_LimeNetwork = new LimeNetwork();
  //private final Pathplanning m_Pathplanning = new Pathplanning();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  //autons
  private final RClear_ScoreTwoAndGrabAuton mRClear_ScoreTwoAndGrabAuton = new RClear_ScoreTwoAndGrabAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);
  private final RClear_ScoreThreeAuton mRClear_ScoreThreeAuton = new RClear_ScoreThreeAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);
  private final RClear_ScoreTwoAndBalanceAuton mRClear_ScoreTwoAndBalanceAuton = new RClear_ScoreTwoAndBalanceAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);
  private final RWire_ScoreTwoCrossAuton mRWire_ScoreTwoCrossAuton = new RWire_ScoreTwoCrossAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);
  private final BClear_ScoreTwoAndGrabAuton mBClear_ScoreTwoAndGrabAuton = new BClear_ScoreTwoAndGrabAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);
  private final BClear_ScoreThreeAuton mBClear_ScoreThreeAuton = new BClear_ScoreThreeAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);
  private final BClear_ScoreTwoAndBalanceAuton mBClear_ScoreTwoAndBalanceAuton = new BClear_ScoreTwoAndBalanceAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);
  private final BWire_ScoreTwoCrossAuton mBWire_ScoreTwoCrossAuton = new BWire_ScoreTwoCrossAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);
  private final Mid_CrossGrabBalanceAuton mCrossGrabBalanceAuton = new Mid_CrossGrabBalanceAuton(m_armSubsystem, m_ClawSubsystem, m_swerveSubsystem);


  
  //arm command
  public final ArmCommand m_ArmCommand = new ArmCommand(m_armSubsystem);

  //claw commands
  public final ClawModeToggleCommand m_ClawCommand = new ClawModeToggleCommand(m_ClawSubsystem);
  public final ClawWheelCommand m_IntakeIn = new ClawWheelCommand(0, m_ClawSubsystem);
  public final ClawWheelCommand m_IntakeOut = new ClawWheelCommand(1, m_ClawSubsystem);

  //server command
  public final SwerveDriveCommand m_swerveDriveCommand = new SwerveDriveCommand(m_swerveSubsystem);
  public final BalanceAuton m_balanceAutonCommand = new BalanceAuton(m_swerveSubsystem);
  public final DriveToPositionAuton m_driveToPoseCommand = new DriveToPositionAuton(0, 0, m_swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(), m_swerveSubsystem);


  // auton chooser
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  public RobotContainer() {
    //DEBUG
    // SmartDashboard.putNumber("Balance P", 0.0);
    // SmartDashboard.putNumber("Balance I", 0.0);
    // SmartDashboard.putNumber("Balance D", 0.0);

    //m_swerveSubsystem.setDefaultCommand(m_swerveDriveCommand);
    m_armSubsystem.setDefaultCommand(m_ArmCommand);
    
    // Initializie auton chooser in smartdashboard
    m_autoChooser.setDefaultOption("Mid Score Grab Balance", mCrossGrabBalanceAuton);
    m_autoChooser.addOption("Blue Clear Score 3", mBClear_ScoreThreeAuton);
    m_autoChooser.addOption("Blue Clear Score 2 Balance", mBClear_ScoreTwoAndBalanceAuton);
    m_autoChooser.addOption("Blue Clear Score 2 Grab", mBClear_ScoreTwoAndGrabAuton);
    m_autoChooser.addOption("Blue Wire Score 2", mBWire_ScoreTwoCrossAuton);
    m_autoChooser.addOption("Red Clear Score 3", mRClear_ScoreThreeAuton);
    m_autoChooser.addOption("Red Clear Score 2 Balance", mRClear_ScoreTwoAndBalanceAuton);
    m_autoChooser.addOption("Red Clear Score 2 Grab", mRClear_ScoreTwoAndGrabAuton);
    m_autoChooser.addOption("Red Wire Score 2", mRWire_ScoreTwoCrossAuton);
    //m_autoChooser.addOption("Blue Balance Auton", m_BlueBalanceAuton);
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