// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {

  public final Pigeon2 m_Pigeon2 = ;

  public int pigeonOffset = 0;

  Translation2d m_frontleftdriveLocation = new Translation2d(1, -1);//need to change to actually coordiantes from the center
  Translation2d m_frontrightdriveLocation = new Translation2d(1, 1);
  Translation2d m_backleftdriveLocation = new Translation2d(-1, -1);//need to change to actually coordiantes from the center
  Translation2d m_backrightdriveLocation = new Translation2d(-1, 1);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontleftdriveLocation, m_frontrightdriveLocation,
  m_backleftdriveLocation, m_backrightdriveLocation);

  public final SwerveModuleState moduleStates = m_kinematics.toSwerveModuleStates(powers);



  public SwerveDriveSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
