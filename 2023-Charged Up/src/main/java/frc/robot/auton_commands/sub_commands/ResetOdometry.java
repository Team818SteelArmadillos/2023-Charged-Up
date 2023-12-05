// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.Vision.Vision;

public class ResetOdometry extends CommandBase {
  /** Creates a new ResetOdometry. */
  Vision m_vision;
  CTRSwerveSubsystem m_drive;
  public ResetOdometry(Vision vision, CTRSwerveSubsystem drive) {
    m_vision = vision;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.getCTRSwerveDrivetrain().setPose(m_vision.getVisionOdometry());


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
