// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;

public class ManualPathPlanning extends SubsystemBase {
  /** Creates a new ManualPathPlanning. */
  public ManualPathPlanning(double goalX, double goalY, double posX, double posY) {
    double slopeX = Math.cos(goalX - posX);
    double slopeY = Math.sin(goalY - posY);
    double distance = Math.sqrt((slopeX * slopeX) + (slopeY * slopeY)); //messed up becuase finding the distance of a unit vector instead of actual vector
    double scale = 1;
    if(distance < 8){
      scale = (Math.pow(1/2 * distance, 1/3));
    }
    Translation2d autonTrans = new Translation2d(slopeX * scale, slopeY * scale);
    



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
