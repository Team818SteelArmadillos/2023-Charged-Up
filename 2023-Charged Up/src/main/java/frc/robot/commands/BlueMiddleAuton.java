// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.subsystems.PistonClawSubsystem;
import frc.robot.subsystems.PivotingArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingArmSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueMiddleAuton extends SequentialCommandGroup {
  /** Creates a new BlueMiddleAuton. */
  private static SwerveDrivetrain m_swerveDrivetrain;
  private static PivotingArmSubsystem m_PivotingArmSubsystem;
  private static TelescopingArmSubsystem m_TelescopingArmSubsystem;
  private static PistonClawSubsystem m_PistonClawSubsystem;
  static double[][] coordinates = {{0, 70.78, -255.11}, {0, 70.78, -172.61}};
  public BlueMiddleAuton(TelescopingArmSubsystem telescopingArmSubsystem, PivotingArmSubsystem pivotingArmSubsystem, 
  SwerveDrivetrain swerveDrivetrain, PistonClawSubsystem pistonClawSubsystem) {

    addRequirements(swerveDrivetrain, pivotingArmSubsystem, telescopingArmSubsystem, pistonClawSubsystem);
    m_TelescopingArmSubsystem = telescopingArmSubsystem;
    m_PivotingArmSubsystem = pivotingArmSubsystem;
    m_swerveDrivetrain = swerveDrivetrain;
    m_PistonClawSubsystem = pistonClawSubsystem;

    new SequentialCommandGroup(
      new ArmAuton(m_PivotingArmSubsystem, m_TelescopingArmSubsystem, 2),
        new ClawCommand(m_PistonClawSubsystem),
      new ArmAuton(m_PivotingArmSubsystem, m_TelescopingArmSubsystem, 3),
      new DriveDistance(m_swerveDrivetrain, new Pose2d(70.78, -172.1, new Rotation2d(0)), true, true)
      );
  }
}

 