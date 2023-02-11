// This file sets up a drivetrain subsystem

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class DriveTrainSubsystem extends SubsystemBase { 

  //motors defines each of the four motors in an array with motors[1] as the first left motor and etc
  static TalonSRX rightMotorOne;// = new TalonSRX(Constants.MOTOR_PORTS_RIGHT[0]);
  static VictorSPX rightMotorTwo;// = new VictorSPX(Constants.MOTOR_PORTS_RIGHT[1]);
  static TalonSRX leftMotorOne;// = new TalonSRX(Constants.MOTOR_PORTS_LEFT[0]);
  static VictorSPX leftMotorTwo;// = new VictorSPX(Constants.MOTOR_PORTS_LEFT[1]);

  static {
    rightMotorOne = new TalonSRX(Constants.MOTOR_PORTS_RIGHT[0]);
    rightMotorTwo = new VictorSPX(Constants.MOTOR_PORTS_RIGHT[1]);
    leftMotorOne = new TalonSRX(Constants.MOTOR_PORTS_LEFT[0]);
    leftMotorTwo = new VictorSPX(Constants.MOTOR_PORTS_LEFT[1]);
    rightMotorOne.setInverted(true);
    rightMotorTwo.setInverted(true);
  }

  public static void setMotorSpeed(double rightSpeed, double leftSpeed){
    rightMotorOne.set(ControlMode.PercentOutput, rightSpeed); //sets the first left motor to leftspeed
    rightMotorTwo.follow(rightMotorOne);
    leftMotorOne.set(ControlMode.PercentOutput, leftSpeed);//sets the first right motor to rightspeed
    leftMotorTwo.follow(leftMotorOne);
  }
}