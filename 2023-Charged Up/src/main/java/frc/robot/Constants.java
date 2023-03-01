// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

   /*============================
               Swerve 
    ==============================*/

    /* CAN IDs */
    public static final int BACK_LEFT_DRIVE = 7; //Josh
    public static final int BACK_LEFT_ENCODER = 11; //Gary 
    public static final int BACK_LEFT_AZIMUTH = 6 ; //Tracy

    public static final int BACK_RIGHT_DRIVE = 4; //Happy
    public static final int BACK_RIGHT_ENCODER = 10; //Bre
    public static final int BACK_RIGHT_AZIMUTH = 5; //Samuel

    public static final int FRONT_RIGHT_DRIVE = 2; //Keith
    public static final int FRONT_RIGHT_ENCODER = 9; //Freddy Mercury
    public static final int FRONT_RIGHT_AZIMUTH = 3; //Beth

    public static final int FRONT_LEFT_DRIVE = 1; //Chad
    public static final int FRONT_LEFT_ENCODER = 8; //Jonathan 
    public static final int FRONT_LEFT_AZIMUTH = 0; //Geraldine

    public static final int THROUGH_BORE_ENCODER = 10;

    /*Can Bus */

    public static final String CAN_BUS_DRIVE = "CANivore";

    public static final int PIGEON = 12;

    /* CANCoder offsets */
    public static double FRONT_LEFT_OFFSET = 284.6; // -158
    public static double FRONT_RIGHT_OFFSET = 172.8; // -286
    public static double BACK_LEFT_OFFSET = 325.2; // -90
    public static double BACK_RIGHT_OFFSET = 100. ; // -197

    public static double FRONT_LEFT_MULTIPLIER = 1; // 
    public static double FRONT_RIGHT_MULTIPLIER = 1; // 
    public static double BACK_LEFT_MULTIPLIER = 1; // 
    public static double BACK_RIGHT_MULTIPLIER = 1; // 

    

    /* Azimuth reversed */
    public static boolean FRONT_LEFT_AZIMUTH_REVERSED = true;
    public static boolean FRONT_RIGHT_AZIMUTH_REVERSED = true;
    public static boolean BACK_LEFT_AZIMUTH_REVERSED = true;
    public static boolean BACK_RIGHT_AZIMUTH_REVERSED = true;

    /* Drive motors reversed */
    public static boolean FRONT_LEFT_DRIVE_REVERSED = true;
    public static boolean FRONT_RIGHT_DRIVE_REVERSED = true;
    public static boolean BACK_LEFT_DRIVE_REVERSED = true;
    public static boolean BACK_RIGHT_DRIVE_REVERSED = true;

    /* CANCoders reversed */
    public static boolean FRONT_LEFT_CANCODER_REVERSED = true;
    public static boolean FRONT_RIGHT_CANCODER_REVERSED = true;
    public static boolean BACK_LEFT_CANCODER_REVERSED = true;
    public static boolean BACK_RIGHT_CANCODER_REVERSED = true;

    /* Gyro reversed */
    public static final boolean INVERT_GYRO = false;

    /* Encoder Ticks per Revolution */
    public static final int AZIMUTH_TICKS_PER_REVOLUTION = 2048; // 21074 withvgear ratio 360 deg
    public static final int DRIVE_TICKS_PER_REVOLUTION = 2048;
    public static final double CODE_LOOPS_PER_MIN = 600.0;


    /* Angle Motor PID Values */
    public static final double AZIMUTH_P = 0.45;
    public static final double AZIMUTH_I = 0.0;
    public static final double AZIMUTH_D = 0.0;
    public static final double AZIMUTH_F = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_P = 0.0; 
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_F = 0.0;

    /* Drive Motor Characterization Values */
    public static final double DRIVE_S = (0.48665 / 12); //Values from SysId divided by 12 to convert to volts for CTRE
    public static final double DRIVE_V = (2.4132 / 12);
    public static final double DRIVE_A = (0.06921 / 12);

    /* Azimuth Current Limiting */
    public static final int AZIMUTH_CONTINUOUS_CURRENT_LIMIT = 25; // amps
    public static final int AZIMUTH_PEAK_CURRENT_LIMIT = 40; // amps
    public static final int AZIMUTH_PEAK_CURRENT_DURATION = 100; // milliseconds
    public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

    /* Drive Current Limiting */
    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35; // amps
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60; // amps
    public static final int DRIVE_PEAK_CURRENT_DURATION = 100; // milliseconds
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* Neutral Modes */
    public static final NeutralMode AZIMUTH_NEUTRAL_MODE = NeutralMode.Coast ;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

    /* Swerve Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = (6.65 / 1.0); //6.86:1 from SDS
    public static final double AZIMUTH_GEAR_RATIO = (10.29/1.0);//(12.8 / 1.0); //12.8:1 from SDS

    /* Swerve Profiling Values */
    public static final double MAX_SPEED = (Units.feetToMeters(17.01)); //meters per second (theoretical from SDS) TBD CHANGED
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12; //radians per second (theoretical calculation)
    public static final double TURN_IN_PLACE_SPEED = 0.5;
    public static final double A_RATE_LIMITER = 2.0; //Slew Rate Limiter Constant

    /* Auto Swerve profiling */
    public static final double AUTO_MAX_SPEED = Units.feetToMeters(17.01);
    public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 3;
    public static final double AUTO_P_X_CONTROLLER = 0.1; 
    public static final double AUTO_P_Y_CONTROLLER = 1.4884;
    public static final double AUTO_P_THETA_CONTROLLER = 2.8;
    public static final double FOUR_BALL_MAX_SPEED = Units.feetToMeters(12.0);
    public static final double FOUR_BALL_MAX_ACCELERATION = 5;
    public static final double WACK_MAX_SPEED = Units.feetToMeters(15.0);
    public static final double WACK_MAX_ACCELERATION = 5;
    
    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            Math.PI, (Math.PI * Math.PI));


    /*============================
               Kinematics
    ==============================*/

    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(26);
    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(26);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0));


    /*============================
                Misc.
    ==============================*/

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

     /*============================
         Controller Constants
    ==============================*/

    /* Controller Constants */
    public static final double STICK_DEADBAND = 0.3;
    public static final int DRIVER_PORT = 0;
    public static final int TEST_PORT = 5;
    public static final double OP_RUMBLE_PERCENT = 0.4;
    public static final double DRIVER_RUMBLE_PERCENT = 0.4;
    public static final RumbleType DRIVER_RUMBLE_LEFT = RumbleType.kLeftRumble;
    public static final RumbleType OP_RUMBLE_LEFT = RumbleType.kLeftRumble;
    public static final RumbleType DRIVER_RUMBLE_RIGHT = RumbleType.kRightRumble;
    public static final RumbleType OP_RUMBLE_RIGHT = RumbleType.kRightRumble;
    public static final double DRIVING_INTAKE_RUMBLE = 0.3;

    //Drive Ports
    
    // Controller Ports
    public static final int operatorControllerPort = 2;
    
    // Arm Motor Ports
    public static final int[] pivotingMotorPorts = {13, 14, 15};
    public static final int telscopingMotorPort = 0;
    public static final int[] clawWheelMotorPort = {0, 0};

    // Operator Values 
    public static final double pP = 0.5;
    public static final double pivotI = 0;
    public static final double pivotD = 1;

    //Telescoping Values
    public static final double armLengths[] = {0, 0, 0, 0, 0};

    public static final double ticksToFeet = 0;
    public static final double tP = 0.5;
    public static final double tI = 0;
    public static final double tD = 1;

    // Piston Claw Values
    public static final int[] clawPistonPort = {0, 17, 1}; // Copied from 2022 Rapid React Code (needs to be tested more??)
    public static final double clawWheelForawrdSpeed = 0.50;
    public static final double clawWheelReverseSpeed = -0.50;



    // Pivoting Arm Angles
    public static final int[] armAngles = {0, 30, 45, 67, 90};

    // Conversion Factors
    public static final double ticksToDegrees = 0.087890625;
    /* 360 degrees divided by 4096 ticks 
    Multiply given ticks by this constant */

    // Neo 550 Motor Stuff
    public static final int neoAmpLimit = 20 /*AMPs*/; // Cannot forget units!
}