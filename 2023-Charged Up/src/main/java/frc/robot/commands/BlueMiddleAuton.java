// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PistonClawSubsystem;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BlueMiddleAuton extends SequentialCommandGroup {
  /** Creates a new BlueMiddleAuton. */
  private static SwerveDrivetrain m_swerveDrivetrain;
  private static PivotingArmSubsystem m_PivotingArmSubsystem;
  private static TelescopingArmSubsystem m_TelescopingArmSubsystem;
  private static PistonClawSubsystem m_PistonClawSubsystem;
  private static LEDSubsystem m_LEDSubsystem;
  static double[] start = {Units.inchesToMeters(70.78), Units.inchesToMeters(-255.11)};
  
  static double[] coordinates = {Units.inchesToMeters(70.78), Units.inchesToMeters(-172.61)};
  
  public BlueMiddleAuton(TelescopingArmSubsystem telescopingArmSubsystem, PivotingArmSubsystem pivotingArmSubsystem, 
  SwerveDrivetrain swerveDrivetrain, PistonClawSubsystem pistonClawSubsystem, LEDSubsystem ledSubsystem) {

    addRequirements(swerveDrivetrain, pivotingArmSubsystem, telescopingArmSubsystem, pistonClawSubsystem);
    m_TelescopingArmSubsystem = telescopingArmSubsystem;
    m_PivotingArmSubsystem = pivotingArmSubsystem;
    m_swerveDrivetrain = swerveDrivetrain;
    m_PistonClawSubsystem = pistonClawSubsystem;
    m_LEDSubsystem = ledSubsystem;
    m_swerveDrivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(70.78), Units.inchesToMeters(-255.11), new Rotation2d(0)));
    addCommands(
      new ArmAuton(m_PivotingArmSubsystem, m_TelescopingArmSubsystem, 2), //sets arm high
      new ParallelCommandGroup(new ClawCommand(m_PistonClawSubsystem, m_LEDSubsystem), new WaitCommand(1)), //Dispenses cone
      new ArmAuton(m_PivotingArmSubsystem, m_TelescopingArmSubsystem, 3), //sets arm to neutral position
      new DriveDistance(m_swerveDrivetrain, 8.0, 0.5, 0.0, 1.0, true, true), //Drives to middle of field
      new DriveDistance(m_swerveDrivetrain, 5.0, 0.3, 0.0, -1.0, true, true) //Balances
      );
  }
}

 