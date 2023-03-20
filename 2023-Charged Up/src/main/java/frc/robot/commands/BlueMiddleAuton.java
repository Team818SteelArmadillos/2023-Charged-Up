// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueMiddleAuton extends SequentialCommandGroup {
  /** Creates a new BlueMiddleAuton. */
  private static SwerveDrivetrain m_swerveDrivetrain;
  static double[][] coordinates = {{0, 70.78, -255.11}, {0, 70.78, -172.61}};
  public BlueMiddleAuton() {
    addRequirements(m_swerveDrivetrain);

    new SequentialCommandGroup(
      new 
    )
  }
}

 