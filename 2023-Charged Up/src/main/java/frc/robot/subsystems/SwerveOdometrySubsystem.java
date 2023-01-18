// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants.*;


public class SwerveOdometrySubsystem extends SubsystemBase {
  public static Timer odometryTimer = new Timer();
  private static double timer;
  
   
  private static void coordinates() {
    timer = odometryTimer.get();
    odometryTimer.stop();
    odometryTimer.reset();
    odometryTimer.start();
    int trueMovement = Math.atan(Robot.m_oi.getleftYAxis()/Robot.m_oi.getleftXAxis());
  }
    private static double[] deltaRobot(){
      double speed1 = timer * ((FRONTRIGHTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed2 = timer * ((FRONTLEFTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed3 = timer * ((BACKRIGHTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed4 = timer * ((BACKLEFTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double angle1 = falconToDegrees(FRONTRIGHTDRIVEMOTOR.getSelectedSensorVelocity());
    }

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
