package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  
  public static class DriveConstants {
    public static final int FRONTRIGHTDRIVEMOTOR = 1;
    public static final int FRONTLEFTDRIVEMOTOR = 2;
    public static final int BACKRIGHTDRIVEMOTOR = 3;
    public static final int BACKLEFTDRIVEMOTOR = 4;
    public static final int BACKUPDRIVEMOTOR = 5;

    public static final int FRONTRIGHTTURNMOTOR = 6;
    public static final int FRONTLEFTTURNMOTOR = 7;
    public static final int BACKRIGHTTURNMOTOR = 8;
    public static final int BACKLEFTTURNMOTOR = 9;
    public static final int BACKUPTURNMOTOR = 10;

    public static final int FRONTRIGHTENCODER = 11;
    public static final int FRONTLEFTENCODER = 12;
    public static final int BACKRIGHTENCODER = 13;
    public static final int BACKLEFTENCODER = 14;

    public static final int PIGEONPORT = 0;

    public static final double GEARRATIO = 6.55;
    public static final int WHEELDIAMETER = 4;
    public static final double WHEELCIRCUMFERENCE = WHEELDIAMETER * Math.PI;
    public static final int PULSEPERREVOLUTION  = 2048;
    public static final double DISTANCEPERPULSE = WHEELCIRCUMFERENCE / PULSEPERREVOLUTION;
    public static final int VELOCITYCALCULATIONPERSECOND = 10;

    public static double degreesToFalcon(double degrees){
      double ticks = degrees/ (360.0/ (GEARRATIO * 2048.0));
      return ticks;
    }
    
    public static double falconToDegrees(double counts){
      return (counts * (360.0 / PULSEPERREVOLUTION))%360;
    }

    public static double falconToRPM(double velocityCounts) {

      double motorRPM = velocityCounts * (600.0 / 2048.0);        
      double mechRPM = motorRPM / GEARRATIO;
      return mechRPM;

    }

      public static double falconToMPS(double velocitycounts, double circumference){
        return falconToRPM(velocitycounts) * circumference / 60;
    }
  }

}
