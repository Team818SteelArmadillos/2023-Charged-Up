// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BlueMiddleAuton extends SequentialCommandGroup {
  /** Creates a new BlueMiddleAuton. */
  private static SwerveDrivetrain m_swerveDrivetrain;
  private static ArmSubsystem m_armSubsystem;
  private static ClawSubsystem m_clawSubsystem;
  private static LEDSubsystem m_LEDSubsystem;
  static double[] start = {Units.inchesToMeters(70.78), Units.inchesToMeters(-255.11)};
  
  static double[] coordinates = {Units.inchesToMeters(70.78), Units.inchesToMeters(-172.61)};
  
  public BlueMiddleAuton(ArmSubsystem armSubsystem, 
  SwerveDrivetrain swerveDrivetrain, ClawSubsystem clawSubsystem, LEDSubsystem ledSubsystem) {

    addRequirements(swerveDrivetrain, armSubsystem, armSubsystem);
    m_armSubsystem = armSubsystem;
    m_swerveDrivetrain = swerveDrivetrain;
    m_clawSubsystem = clawSubsystem;
    m_LEDSubsystem = ledSubsystem;
    m_swerveDrivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(70.78), Units.inchesToMeters(-255.11), new Rotation2d(0)));
    addCommands(
      new ArmAuton(m_armSubsystem, 2), //sets arm high
      new WaitCommand(0.5),
      new ParallelCommandGroup(new ClawCommand(m_clawSubsystem, m_LEDSubsystem), new WaitCommand(0.8)), //Dispenses cone
      new ArmAuton(m_armSubsystem, 3), //sets arm to neutral position
      new DriveDistance(m_swerveDrivetrain, 8.25, 0.7, 0.0, 1.0, true, true), //Drives to middle of field
      new WaitCommand(0.8),
      new DriveToRamp(m_swerveDrivetrain, 0.5, 0.0, -1.0, true, true), //Balances
      new DriveToBalance(m_swerveDrivetrain, 0.105, 0.0, -1.0, true, true), //Balances
      //new AutoBalanceCommand(m_swerveDrivetrain, m_LEDSubsystem),
      new HoldPosition(m_swerveDrivetrain)
      );
  }
}

 