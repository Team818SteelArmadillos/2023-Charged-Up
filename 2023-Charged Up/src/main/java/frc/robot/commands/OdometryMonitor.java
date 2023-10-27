// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.Vision.Vision;

public class OdometryMonitor extends CommandBase {
  Vision m_Vision;
  CTRSwerveSubsystem m_CtrSwerveSubsystem;
  Pose2d visionOdometry;
  /** Creates a new OdometryMonitor. */
  public OdometryMonitor(Vision vision, CTRSwerveSubsystem ctrSwerveSubsystem) {
    addRequirements(vision, ctrSwerveSubsystem);
    m_Vision = vision;
    m_CtrSwerveSubsystem = ctrSwerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Vision.getActivePipeline()!= 0){
      m_Vision.setAprilTag();
    }
    
    m_Vision.updateVisionOdometry();
    visionOdometry = m_Vision.getVisionOdometry();

    //post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight BotPose X", visionOdometry.getX());
    SmartDashboard.putNumber("Limelight BotPose X", visionOdometry.getY());
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
