// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenixpro.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.CTRSwerve.SwerveDriveConstantsCreator;
import frc.robot.CTRSwerve.SwerveDriveTrainConstants;
import frc.robot.CTRSwerve.SwerveModuleConstants;

public class CTRSwerveSubsystem extends SubsystemBase {

  private SwerveDriveTrainConstants drivetrain;
  private Slot0Configs steerGains;
  private Slot0Configs driveGains;
  private SwerveDriveConstantsCreator m_constantsCreator;
  private SwerveModuleConstants frontRight;
  private SwerveModuleConstants frontLeft;
  private SwerveModuleConstants backRight;
  private SwerveModuleConstants backLeft;
  private CTRSwerveDrivetrain m_drivetrain;
public Supplier<Pose2d> getPose2d;

  /** Creates a new CTRESwerveSubsystem. */
  public CTRSwerveSubsystem() {
    //CTRE Swerve
    drivetrain = new SwerveDriveTrainConstants().withPigeon2Id(Constants.PIGEON_ID).withCANbusName(Constants.CAN_BUS_DRIVE);
    drivetrain.withTurnKp(Constants.DRIVE_TURN_KP);
    drivetrain.withTurnKd(Constants.DRIVE_TURN_KD);

    steerGains = new Slot0Configs();
      steerGains.kP = Constants.STEER_GAINS_KP;
      steerGains.kD = Constants.STEER_GAINS_KD;

    driveGains = new Slot0Configs();
      driveGains.kP = Constants.DRIVE_GAINS_KP;
      
    m_constantsCreator = new SwerveDriveConstantsCreator(
      Constants.DRIVE_GEAR_RATIO, 
      Constants.AZIMUTH_GEAR_RATIO, 
      Constants.WHEEL_RADIUS_INCHES, 
      Constants.SLIP_CURRENT, 
      steerGains, 
      driveGains, 
      true
      );

    frontRight = m_constantsCreator.createModuleConstants(
      Constants.FRONT_RIGHT_AZIMUTH, 
      Constants.FRONT_RIGHT_DRIVE, 
      Constants.FRONT_RIGHT_CANCODER, 
      Constants.FRONT_RIGHT_OFFSET, 
      Constants.DRIVETRAIN_LENGTH/2.0, 
      -Constants.DRIVETRAIN_LENGTH/2.0
      );
    
    frontLeft = m_constantsCreator.createModuleConstants(
      Constants.FRONT_LEFT_AZIMUTH, 
      Constants.FRONT_LEFT_DRIVE, 
      Constants.FRONT_LEFT_CANCODER, 
      Constants.FRONT_LEFT_OFFSET, 
      Constants.DRIVETRAIN_LENGTH/2.0, 
      Constants.DRIVETRAIN_LENGTH/2.0);

    backRight = m_constantsCreator.createModuleConstants(
      Constants.BACK_RIGHT_AZIMUTH, 
      Constants.BACK_RIGHT_DRIVE, 
      Constants.BACK_RIGHT_CANCODER, 
      Constants.BACK_RIGHT_OFFSET, 
      -Constants.DRIVETRAIN_LENGTH/2.0, 
      -Constants.DRIVETRAIN_LENGTH/2.0);

    backLeft = m_constantsCreator.createModuleConstants(
      Constants.BACK_LEFT_AZIMUTH, 
      Constants.BACK_LEFT_DRIVE, 
      Constants.BACK_LEFT_CANCODER, 
      Constants.BACK_LEFT_OFFSET, 
      -Constants.DRIVETRAIN_LENGTH/2.0, 
      Constants.DRIVETRAIN_LENGTH/2.0);

    m_drivetrain = new CTRSwerveDrivetrain(drivetrain, frontRight, frontLeft, backRight, backLeft);

    m_drivetrain.seedFieldRelativeButBackwards(); // WE START BACKWARDS
  }

  public CTRSwerveDrivetrain getCTRSwerveDrivetrain() {
    return m_drivetrain;
  }

  public void setPose(Pose2d pose){
    m_drivetrain.setPose(pose);
  }

  public void setChasisSpeeds(ChassisSpeeds speeds){
    m_drivetrain.driveRobotCentric(speeds);
  }
  
  public Pose2d getPose(){
    return m_drivetrain.getPoseMeters();
  }
  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}
