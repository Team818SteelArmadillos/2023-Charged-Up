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
                Arm 
    ==============================*/

    public static final int THROUGH_BORE_ENCODER = 0;

    // pivoting motor Values 
    public static final double pP = -0.025;
    public static final double pivotI = 0.0;
    public static final double pivotD = 0.0;
    public static final double pPIDTolerance = 2.0;

    //Telescoping Values
    public static final double armLengths[] = {0, 50000, 98000, Constants.maximumArmLength, 167500};
    public static final double maximumArmLength = 167500; //
    public static final double minimumArmLength = 0; //
    public static final double lengthSlewRate = 20000;
    public static final int limitSwitchPort = 0;

    public static final double ticksToFeet = 0;
    public static final double tP = 0.10;
    public static final double tI = 0.000005;
    public static final double tD = 0;
    public static final double tTolerance = 600;

    // Pneumatic ports
    public static final int pneumaticPistonPort = 17; // Copied from 2022 Rapid React Code (needs to be tested more??)
    public static final int[] pneumaticPorts = {0,1,2,3,4,5,6,7};

    // Piston Claw Values
    public static final double CONE_IN_SPEED = 0.65;
    public static final double CONE_OUT_SPEED= -0.8;
    public static final double CUBE_IN_SPEED = 0.6;
    public static final double CUBE_OUT_SPEED = -0.2;

    // Pivoting Arm Angles
    public static final int[] armAngles = {0, -37, -52, -54, -98, 90};
    public static final int[] revArmAngles = {0, 37, 50, 98, -90};
    public static final int pivotHardLimit = 115; //this is the maximum angle in degrees that the arm should go (with 0 being vertical)
    public static final double angleSlewRate = 78;
    public static final int armSetpointCounter = 20;
    public static final double controllerDeadzone = 0.1;

    public static final double encoderOvershoot = 0.847;

    // Neo 550 Motor Stuff
    public static final int neoAmpLimit = 20 /*AMPs*/; // Cannot forget units!

    //Falcon 500 Motor Limit
    //public static final int falcon500MaxLimit = x; /*rotations - x is a placeholder */
    //public static final int falcon500MinLimit = y; 
    public static final int boreEncoder = 0;

   /*============================
               Swerve 
    ==============================*/

    /* CAN IDs */
    public static final int BACK_LEFT_DRIVE = 7; //Josh
    public static final int BACK_LEFT_ENCODER = 11; //Gary 
    public static final int BACK_LEFT_AZIMUTH = 6 ; //Tracy
    public static final double BACK_LEFT_OFFSET = 150.29296875;

    public static final int BACK_RIGHT_DRIVE = 4; //Happy
    public static final int BACK_RIGHT_ENCODER = 10; //Bre
    public static final int BACK_RIGHT_AZIMUTH = 5; //Samuel
    public static final double BACK_RIGHT_OFFSET = 264.638671875;

    public static final int FRONT_RIGHT_DRIVE = 2; //Keith
    public static final int FRONT_RIGHT_ENCODER = 9; //Freddy Mercury
    public static final int FRONT_RIGHT_AZIMUTH = 3; //Beth
    public static final double FRONT_RIGHT_OFFSET = 352.44140625;

    public static final int FRONT_LEFT_DRIVE = 1; //Chad
    public static final int FRONT_LEFT_ENCODER = 8; //Jonathan 
    public static final int FRONT_LEFT_AZIMUTH = 0; //Geraldine
    public static final double FRONT_LEFT_OFFSET = 108.193359375;

    public static final double PIGEON_ROLL_OFFSET = 1.8;

    /*Can Bus */
    public static final String CAN_BUS_DRIVE = "CANivore";

    public static final int PIGEON = 12;

    /* CANCoder offsets */
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
    public static final int AZIMUTH_TICKS_PER_REVOLUTION = 2048;
    public static final int DRIVE_TICKS_PER_REVOLUTION = 2048;
    public static final double CODE_LOOPS_PER_MIN = 600.0;

    /* Angle Motor PID Values */
    public static final double AZIMUTH_P = 0.45;
    public static final double AZIMUTH_I = 0.0;
    public static final double AZIMUTH_D = 0.0;
    public static final double AZIMUTH_F = 0.0;

    //Rotation PID Controller values
    public static final double ROTATION_P = 0.0025;
    public static final double ROTATION_I = 0.00005;
    public static final double ROTATION_D = 0.0;
    public static final double ROTATION_TOLERANCE = 1.0;

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
    public static final int AZIMUTH_CONTINUOUS_CURRENT_LIMIT = 10; // amps
    public static final int AZIMUTH_PEAK_CURRENT_LIMIT = 20; // amps
    public static final int AZIMUTH_PEAK_CURRENT_DURATION = 100; // milliseconds
    public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

    /* Drive Current Limiting */
    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 30; // amps
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 40; // amps
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

    public static final int CANDLE_CAN_ID = 18;

    public static final double MINIMUM_CHARGE_STATION_ANGLE_THRESH = 10.0;
    public static final double MINIMUM_INCLINE_THRESHOLD = 1;

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;
    public static final double LIMELIGHT_GROUNDOFFSET = 20.75;

    //PID values for Charging Station
    public static final double csP = 0.02;
    public static final double csI = 0;
    public static final double csD = 0;
    public static final double csTolerance = 5;

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
    public static final int leftYAxis = 1;
    public static final int rightYAxis = 5;
    
    // Controller Ports
    public static final int operatorControllerPort = 2;
    
    // Arm Motor Ports
    public static final int[] pivotingMotorPorts = {13, 14, 15};
    public static final int telscopingMotorPort = 16;
    public static final int[] clawWheelMotorPort = {20, 21};

}