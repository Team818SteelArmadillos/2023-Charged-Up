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

public class AutoDrive extends SubsystemBase {
    m_swerveDriveSubsystem.drive(autonTrans, rotation, true, true);
  }
  public Translation2d autoDrive(double goalX, double goalY, double posX, double posY){
   //Finding distance from position to target
   double distance = Math.sqrt(Math.pow(goalX - posX, 2) + Math.pow(goalY - posY, 2));
   //Converting slope to unit vector for controller values
   double slopeX = Math.cos(goalX - posX);
   double slopeY = Math.sin(goalY - posY);
   //Filtering distance to prevent overrun. If over/underrun change equation or value 8 to increase/decrease time at full speed.
   double scale = distance < 8 ? (Math.pow(1/2 * distance, 1/3)) : 1; 
   Translation2d autonTrans = new Translation2d(slopeX * scale, slopeY * scale);
    return autonTrans;
  }
  public double autorotate(double goalAng){
    double rotation;
    //Finds difference in current and goal angle value, if any, to rotate robot appropriately
    double difAngle = SwerveDrivetrain.getYaw().getDegrees() - goalAng;
    if(difAngle == 0){
      rotation = 0;
    }else{
      rotation = difAngle > 0 ? -(Math.pow(5, -0.2*(difAngle-7.5)) - Constants.MAX_ANGULAR_VELOCITY): -(Math.pow(5, 0.2*(difAngle+7.5)) - Constants.MAX_ANGULAR_VELOCITY);
    }
    return rotation;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
