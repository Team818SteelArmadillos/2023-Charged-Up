// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.Vision;

public class OdometryMonitor extends CommandBase {
  Vision m_Vision;
  Pose2d visionOdometry;

  int counter;

  /** Creates a new OdometryMonitor. */
  public OdometryMonitor(Vision vision) {
    addRequirements(vision);
    m_Vision = vision;
    counter = 0;
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
    
    m_Vision.updateVisionOdometry(RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain());


    //post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight BotPose X", m_Vision.getVisionOdometry().getX());
    SmartDashboard.putNumber("Limelight BotPose Y", m_Vision.getVisionOdometry().getY());
    SmartDashboard.putNumber("Limelight BotPose Rotation", m_Vision.getVisionOdometry().getRotation().getDegrees());
    SmartDashboard.putNumber("Odometry innacuracy counter", counter);
    
    
    Logger.getInstance().recordOutput("Vision2", new Pose2d(m_Vision.getVisionOdometry().getX(), m_Vision.getVisionOdometry().getY(), m_Vision.getVisionOdometry().getRotation()));
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
