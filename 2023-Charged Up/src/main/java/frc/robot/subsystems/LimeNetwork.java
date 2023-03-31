// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeNetwork extends SubsystemBase {

  // Limelight network table
  private NetworkTable limeTable;

  // Limelight network entries
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry tBotPose;

  private CTRSwerveSubsystem m_drivetrain;

  public LimeNetwork() {
    limeTable = NetworkTableInstance.getDefault().getTable("limelight");
    
    m_drivetrain = new CTRSwerveSubsystem();

    tx = limeTable.getEntry("tx");
    ty = limeTable.getEntry("ty");
    tBotPose = limeTable.getEntry("botpose");
  
  }

  public  double getTX() {
    return tx.getDouble(0.0);
  }
  
  public  double getTY() {
    return ty.getDouble(0.0);
  }

  public double getXOffset() {
    return getRobotPose()[0] - m_drivetrain.getCTRSwerveDrivetrain().getPoseMeters().getX();
  }

  public double getYOffset() {
    return getRobotPose()[1] - m_drivetrain.getCTRSwerveDrivetrain().getPoseMeters().getY();
  }

  public double[] getRobotPose() {
    return tBotPose.getDoubleArray( new double[] {0.0, 0.0, 0.0} );
  }

}