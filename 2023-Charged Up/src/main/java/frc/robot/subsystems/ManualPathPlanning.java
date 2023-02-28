// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class ManualPathPlanning extends SubsystemBase {
  /** Creates a new ManualPathPlanning. */
  public ManualPathPlanning(double goalX, double goalY, double posX, double posY, double goalAng) {
    double norX = Math.cos(goalX - posX);
    double norY = Math.sin(goalY - posY);

    double slopeX = Math.cos(goalX - posX);
    double slopeY = Math.sin(goalY - posY);
    double distance = Math.sqrt(Math.sqrt(norX) + Math.sqrt(norY)); 
  
    double scale = distance < 8 ? (Math.pow(1/2 * distance, 1/3)) : 1;

    double rotation;
    double difAngle = SwerveDrivetrain.getYaw().getDegrees() - goalAng;
    if(difAngle == 0){
      rotation = 0;
    }else{
      rotation = difAngle > 0 ? -(Math.pow(5, -0.2*(difAngle-7.5)) - Constants.MAX_ANGULAR_VELOCITY): -(Math.pow(5, 0.2*(difAngle+7.5)) - Constants.MAX_ANGULAR_VELOCITY);
    }

    Translation2d autonTrans = new Translation2d(slopeX * scale, slopeY * scale);

    m_swerveDriveSubsystem.drive(autonTrans, rotation, true, true);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
