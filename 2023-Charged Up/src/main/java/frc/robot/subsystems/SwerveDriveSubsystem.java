package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveModuleSubsytem.SwerveModule;

public class SwerveDriveSubsystem extends SubsystemBase {
  public SwerveDriveOdometry m_swerveodometry;
  public SwerveModule[] m_swerveModules;
  public SwerveModuleState[] m_swerveModuleStates;
  Pigeon2 m_pigeon2 = new Pigeon2(1);
  SwerveModuleState moduleState;
  
  
  
  

  public SwerveDriveSubsystem() {
  // replace the 1 with the port number of the pigeon2
  m_swerveodometry = new SwerveDriveOdometry(DriveConstants.swerveKinematics, getYaw(), getPosition(), new Pose2d(5.0, 13.5, new Rotation2d()));// place in the yaw and kinematics
  
  
  }

  
  public void drive(Translation2d translation, double rotation, boolean field, boolean isOpenLoop){

    SwerveModuleState[] swerveModuleStates = 
      ModuleConstants.swerveKinematics.toSwerveModuleStates(
        field ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
          : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 11.5);// replace 1 with max speed

    for(SwerveModule module : m_swerveModules){
      Rotation2d fillerRotation = new Rotation2d();
      module.setDesiredState(fillerRotation, 1, false);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredState){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 11.5);//replace 1 with max speed

    for(SwerveModule mod: m_swerveModules){
      mod.setDesiredState(null, 0, false);
    }
  }
  



  public void setChassisSpeeds(ChassisSpeeds targetSpeeds){
    setModuleStates(ModuleConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds));
  }

  public Pose2d getPose(){
    return m_swerveodometry.getPoseMeters();//doesn't exist yet because it isn't made
  }
  public SwerveModulePosition[] getPosition(){
    SwerveModulePosition m_frontLeftSMlocation = new SwerveModulePosition(-0.6223, new Rotation2d());
    SwerveModulePosition m_frontRightSMlocation = new SwerveModulePosition(0.6223, new Rotation2d());
    SwerveModulePosition m_backLeftSMlocation = new SwerveModulePosition(-0.6223, new Rotation2d());
    SwerveModulePosition m_backrightSMlocation = new SwerveModulePosition(0.6223, new Rotation2d());
    SwerveModulePosition[] SwerveModulePositionArray = {m_frontLeftSMlocation, m_frontRightSMlocation, m_backLeftSMlocation, m_backrightSMlocation}; 
    return SwerveModulePositionArray;
  }

  public void resetOdometry(Pose2d pose){
    m_swerveodometry.resetPosition(new Rotation2d(),getPosition(),pose);
  }

  public double getAngle(){
    double angle = m_pigeon2.getYaw();  
    return angle;
  }

  public double getNonContinuousGyro(){
    return getAngle() % 360;
  }

  public SwerveModuleState[] getStates(){
    SwerveModuleState[] m_swerveModuleStates = new SwerveModuleState[4];

    for(SwerveModuleState moduleState : m_swerveModuleStates){
      moduleState = SwerveModuleSubsytem.SwerveModule.getDesiredState();
    }
    return m_swerveModuleStates;
  }

  public Rotation2d getYaw() {
    return (Constants.kGyroReversed) ? Rotation2d.fromDegrees(360 - (m_pigeon2.getYaw())) : Rotation2d.fromDegrees(m_pigeon2.getYaw());
  }

  public void resetGyro() {
    m_pigeon2.setYaw(0);
  }

  public void zeroModules(){
     for(SwerveModule mod : m_swerveModules){
      mod.zeroModule();
     }
  }

  public boolean optimizeTurning(double currentAngle, double desiredAngle){

    boolean isDesiredPositive = desiredAngle > 0;
    boolean isCurrentPositive = currentAngle > 0;

    double m_desiredAngle = Math.abs(desiredAngle);
    double m_currentAngle = Math.abs(currentAngle);

    double absTotal = Math.abs(currentAngle) + Math.abs(desiredAngle);

    if(isDesiredPositive && isCurrentPositive){
      if(m_desiredAngle > m_currentAngle){
        return true;
      } else {
        return false;
      }
    } else {
      if(absTotal > 180){
        return false;
      } else {
        return false;
      }
    }

  }

  public SwerveModule[] getModules(){
    return m_swerveModules;
  }


  public void setGyro(double yaw){
    double yawMod;
    if(yaw < 0 ){
      yawMod = 360 - yaw;
    } else if (yaw > 0){
      yawMod = yaw;
    } else {
      yawMod = yaw;
    }

    m_pigeon2.setYaw(yawMod);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
