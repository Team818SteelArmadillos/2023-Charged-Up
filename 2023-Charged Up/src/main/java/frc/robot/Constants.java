package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  public static final class ModuleConstants {

    public static final double kDriveGearRatio = 7.13;

    public static final double kPModuleTurnController = 8.1; //8.3 // TUNE: 8.2142
    public static final double kIModuleTurnController = 0; // DO NOT USE
    public static final double kDModuleTurnController = 0; // TUNE

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 6 * Math.PI;

    public static final double kPModuleDriveController = 1; // TUNE
    public static final double kIModuleDriveController = 0; // DO NOT USE
    public static final double kDModuleDriveController = 0;


    public static final int kDriveFXEncoderCPR = 2048;
    public static final int kTurningCANcoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.1016; // 4 inches
    public static final double kWheelCircumferenceMeters =
        kWheelDiameterMeters * Math.PI; // C = D * pi
    public static final double kDrivetoMetersPerSecond =
        (10 * kWheelCircumferenceMeters) / (kDriveGearRatio * 2048);
  }

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

    public static final double kTrackWidth = 0.57785;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.57785;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;


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

  public static double falconToRPM(double velocityCounts, double gearRatio) {

    double motorRPM = velocityCounts * (600.0 / 2048.0);        
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;

}

  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){

    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;

}


  public static double rpmToFalcon(double RPM, double gearRatio){
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0/ 600.0);
    return sensorCounts;
  }


  public static double mpsToFalcon(double velocity, double circumference, double gearRatio){
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = rpmToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }
} 
