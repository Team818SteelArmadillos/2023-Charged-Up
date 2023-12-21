// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometryMonitoringSubsystem extends SubsystemBase {
  /** Creates a new OdometryMonitoringSubsystem. */
  double xOffSet;
  double yOffSet;
  double rOffSet;

  public OdometryMonitoringSubsystem() {
    double xOffSet = 0;
    double yOffSet = 0;
    double rOffSet = 0;
  }

  public Pose2d robotPose(Pose2d odometryPose, Pose2d visionPose){
    xOffSet+= (1/(2*Math.abs((odometryPose.getX() + xOffSet) - visionPose.getX()) + 1)) * (visionPose.getX() - odometryPose.getX());
    yOffSet+= (1/(2*Math.abs((odometryPose.getY() + yOffSet) - visionPose.getY()) + 1)) * (visionPose.getY() - odometryPose.getY());
    rOffSet+= (1/(2*Math.abs((odometryPose.getRotation().getRadians() + yOffSet) - visionPose.getRotation().getRadians()) + 1)) * (visionPose.getRotation().getRadians() - odometryPose.getRotation().getRadians());

    return new Pose2d(new Translation2d(odometryPose.getX() + xOffSet, odometryPose.getY() + yOffSet), new Rotation2d(odometryPose.getRotation().getRadians() + rOffSet));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
