// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  VictorSPX IntakeExtendMotor, IntakeMotor;
  DoubleSolenoid IntakePistonLock;

  private int lock_counter;

    /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    IntakeExtendMotor = new VictorSPX(Constants.INTAKE_EXTEND_MOTOR_PORT);
    IntakeMotor = new VictorSPX(Constants.INTAKE_MOTOR_PORT);

    VictorSPXConfiguration victorSPXConfiguration = new VictorSPXConfiguration();

    IntakeExtendMotor.configAllSettings(victorSPXConfiguration);
    IntakeMotor.configAllSettings(victorSPXConfiguration);

    IntakePistonLock = new DoubleSolenoid(Constants.IntakePistonPort, PneumaticsModuleType.CTREPCM, Constants.intakePneumaticPorts[2], Constants.intakePneumaticPorts[1]);
    IntakePistonLock.set(DoubleSolenoid.Value.kReverse);

    lock_counter = 0;

  }

  public void setIntakeSpeed(double speed){
    IntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void intakeUnlock(){
    IntakePistonLock.set(DoubleSolenoid.Value.kForward);
  }

  public void intakeLock(){
    IntakePistonLock.set(DoubleSolenoid.Value.kReverse);
  }

  public void intakeExtend() {
    IntakeExtendMotor.set(ControlMode.PercentOutput, 0.6);
    
    //unlock the intake
    intakeUnlock();
    lock_counter = 0;
  }

  public void intakeRetract() {
    IntakeExtendMotor.set(ControlMode.PercentOutput, 0.0);

    // wait a bit after we retact the intake to lock
    lock_counter++;
    if (lock_counter >=30) {
      lock_counter = 30;
      intakeLock();
    }
  }

  public boolean isIntakeUnlocked(){
    if(IntakePistonLock.get().equals(DoubleSolenoid.Value.kForward)){
      return true;
    } else{
      return false;
    }
  }

  
  @Override
  public void periodic() {
    if (isIntakeUnlocked()) {
      SmartDashboard.putString("IntakePosition", "OUT");
    } else {
        SmartDashboard.putString("IntakePosition", "IN");
    }
    // This method will be called once per scheduler run
  }
}
