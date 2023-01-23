// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Controller Ports
    public static final int driverControllerPort = 1;
    public static final int operatorControllerPort = 2;
    
    // Arm Motor Ports
    public static final int[] pivotingMotorPorts = {0, 0, 0};
    public static final int telscopingMotorPort = 0;
    public static final int[] clawWheelMotorPort = {0, 0};

    // Operator Values 
    public static final double pP = 0.5;
    public static final double pivotI = 0;
    public static final double pivotD = 1;

    //Telescoping Values
    public static final double minArmLength = 0;
    public static final double ticksToFeet = 0;
    public static final double tP = 0.5;
    public static final double tI = 0;
    public static final double tD = 1;

    
    // Conversion Factors
    public static final double ticksToDegrees = 0.087890625; 
    /* 360 degrees divided by 4096 ticks 
    Multiply given ticks by this constant */
}