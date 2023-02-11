// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeNetwork extends SubsystemBase {

  private static NetworkTable limeTable;
  private static NetworkTableEntry tx = limeTable.getEntry("tx");
  private static NetworkTableEntry ty = limeTable.getEntry("ty");
  private static NetworkTableEntry ta = limeTable.getEntry("ta");

  //read values periodically
  private static double x = tx.getDouble(0.0);
  private static double y = ty.getDouble(0.0);
  private static double area = ta.getDouble(0.0);

  static {
    limeTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public static double[] values() {
    double[] values = {x, y, area};
    return values;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
