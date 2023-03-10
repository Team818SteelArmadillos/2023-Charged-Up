// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeNetwork extends SubsystemBase {

  // Limelight network table
  private NetworkTable limeTable;

  // Limelight network entries
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  //read values periodically
  private double x = 0.0;    
  private double y = 0.0;    
  private double area = 0.0; 

  public LimeNetwork() {
    limeTable = NetworkTableInstance.getDefault().getTable("limelight");

    tx = limeTable.getEntry("tx");
    ty = limeTable.getEntry("ty");
    ta = limeTable.getEntry("ta");
  }

  public void updateValues() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
  }

  public  double getTX() {
    updateValues();
    return x;
  }
  
  public  double getTY() {
    updateValues();
    return y;
  }

  public  double getTA() {
    updateValues();
    return area;
  }

  public  double[] values() {
    updateValues();
    double[] values = {x, y, area};
    return values;
  }

}