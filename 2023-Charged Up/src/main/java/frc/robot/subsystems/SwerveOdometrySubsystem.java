// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< HEAD
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveOdometrySubsystem extends SubsystemBase {

  //public final Pigeon2 m_Pigeon2 = ;

  public int pigeonOffset = 0;

  Translation2d m_frontleftdriveLocation = new Translation2d(-10.62, 10.62);//need to change to actually coordiantes from the center
  Translation2d m_frontrightdriveLocation = new Translation2d(10.62, 10.62);
  Translation2d m_backleftdriveLocation = new Translation2d(-10.62, -10.62);//need to change to actually coordiantes from the center
  Translation2d m_backrightdriveLocation = new Translation2d(10.62, -10.62);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontleftdriveLocation, m_frontrightdriveLocation,
  m_backleftdriveLocation, m_backrightdriveLocation);

  public final SwerveModuleState moduleStates = m_kinematics.toSwerveModuleStates(powers);



  public SwerveOdometrySubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
=======
import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants.*;


public class SwerveOdometrySubsystem extends SubsystemBase {
  public static Timer odometryTimer = new Timer();
  private static double timer;
  static double[] coordinates = {0,0};

  public SwerveOdometrySubsystem(){
  }
  
   
  public static double[] getCoordinates() {
    return coordinates;
  }
    private static double[] deltaRobot(){
      double speed1 = timer * ((FRONTRIGHTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed2 = timer * ((FRONTLEFTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed3 = timer * ((BACKRIGHTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed4 = timer * ((BACKLEFTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double angle1 = falconToDegrees(FRONTRIGHTTURNMOTOR.getSelectedSensorVelocity()) + Math.toDegrees(PIGEON.getAngle());
      double angle2 = falconToDegrees(FRONTLEFTTURNMOTOR.getSelectedSensorVelocity()) + Math.toDegrees(PIGEON.getAngle());
      double angle3 = falconToDegrees(BACKRIGHTTURNMOTOR.getSelectedSensorVelocity()) + Math.toDegrees(PIGEON.getAngle());
      double angle4 = falconToDegrees(BACKLEFTTURNMOTOR.getSelectedSensorVelocity()) + Math.toDegrees(PIGEON.getAngle());

      double deltaX = ((speed1 *Math.cos(Math.toRadians(angle1))) + (speed2 * Math.cos(Math.toRadians(angle2))) + (speed3 *
      Math.cos(Math.toRadians(angle3))) + (speed4 * Math.cos(Math.toRadians(angle4))))/4;

      double deltaY = ((speed1 *Math.sin(Math.toRadians(angle1))) + (speed2 * Math.sin(Math.toRadians(angle2))) + (speed3 *
      Math.sin(Math.toRadians(angle3))) + (speed4 * Math.sin(Math.toRadians(angle4))))/4;

      double delta[] = {deltaX, deltaY};
      return delta;
    }
  
  @Override
  public void periodic() {
    timer = odometryTimer.get();
    odometryTimer.stop();
    odometryTimer.reset();
    odometryTimer.start();
    double[] delta = deltaRobot();
    coordinates[0]+=delta[0];
    coordinates[1]+=delta[1];
>>>>>>> d1dbb523443519dcd5c07642069adf1dab0bb377
  }
}
