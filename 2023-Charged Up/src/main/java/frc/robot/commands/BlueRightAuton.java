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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueRightAuton extends SequentialCommandGroup {
  /** Creates a new BlueMiddleAuton. */
  private static SwerveDrivetrain m_swerveDrivetrain;
  private static PivotingArmSubsystem m_PivotingArmSubsystem;
  private static TelescopingArmSubsystem m_TelescopingArmSubsystem;
  private static PistonClawSubsystem m_PistonClawSubsystem;
  private static LEDSubsystem m_LEDSubsystem;
  static double[][] coordinates = {{0, 70.78, -255.11}, {0, 70.78, -172.61}};
  public BlueRightAuton(TelescopingArmSubsystem telescopingArmSubsystem, PivotingArmSubsystem pivotingArmSubsystem, 
  SwerveDrivetrain swerveDrivetrain, PistonClawSubsystem pistonClawSubsystem, LEDSubsystem ledSubsystem) {

    addRequirements(swerveDrivetrain, pivotingArmSubsystem, telescopingArmSubsystem, pistonClawSubsystem);
    m_TelescopingArmSubsystem = telescopingArmSubsystem;
    m_PivotingArmSubsystem = pivotingArmSubsystem;
    m_swerveDrivetrain = swerveDrivetrain;
    m_PistonClawSubsystem = pistonClawSubsystem;
    m_LEDSubsystem = ledSubsystem;

    addCommands(
      new DriveDistance(m_swerveDrivetrain, new Pose2d(121.61, -255.11, new Rotation2d(Math.toRadians(90))), true, true),
      new ArmAuton(m_PivotingArmSubsystem, m_TelescopingArmSubsystem, 2), // sets arm to high position
      new ClawCommand(m_PistonClawSubsystem, m_LEDSubsystem), //drops cone
      new ArmAuton(m_PivotingArmSubsystem, m_TelescopingArmSubsystem, 3), // sets arm to neutral position
      new DriveDistance(swerveDrivetrain, new Pose2d(121.61, -64.62, new Rotation2d(0)), true, true),
      new ArmAuton(m_PivotingArmSubsystem, m_TelescopingArmSubsystem, 0),
      new DriveDistance(swerveDrivetrain, new Pose2d(121.61, -62.62, new Rotation2d(0)), true, true)
        

      );
  }
}

 
