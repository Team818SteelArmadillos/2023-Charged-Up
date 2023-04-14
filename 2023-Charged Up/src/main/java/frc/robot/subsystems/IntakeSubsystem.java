// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  TalonSRX IntakeExtendMotor;
  CANSparkMax IntakeMotor;
  DoubleSolenoid IntakePistonLock, IntakePistonUnused;

  private boolean intakeOut;

    /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    IntakeExtendMotor = new TalonSRX(Constants.INTAKE_EXTEND_MOTOR_PORT);
    IntakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_PORT, MotorType.kBrushless);

    IntakePistonLock = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[0], Constants.pneumaticPorts[1]);
    IntakePistonLock.set(DoubleSolenoid.Value.kForward);
    IntakePistonUnused = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[2], Constants.pneumaticPorts[3]);
    IntakePistonUnused.set(DoubleSolenoid.Value.kForward);
    IntakeExtendMotor.setSelectedSensorPosition(0.0);
    IntakeExtendMotor.config_kP(0, 0.025);

    IntakeMotor.setSmartCurrentLimit(20);

    intakeOut = false;

  }

  public void setIntakeSpeed(double speed){
    IntakeMotor.set(speed);
  }

  public void intakeUnlock(){
    //IntakePistonLock.set(DoubleSolenoid.Value.kForward);
    intakeOut = true;
  }

  public void intakeLock(){
    //IntakePistonLock.set(DoubleSolenoid.Value.kReverse);
    intakeOut = false;
  }

  public void intakeExtend() {
    IntakeExtendMotor.set(ControlMode.Position, -539000);
    
    //unlock the intake
    intakeUnlock();
    //lock_counter = 0;
  }

  public void firePistons(){

  }

  public void intakeRetract() {
    IntakeExtendMotor.set(ControlMode.Position, 0);

    // wait a bit after we retact the intake to lock
    // lock_counter++;
    // if (lock_counter >=30) {
    //   lock_counter = 30;
      intakeLock();
    //}
  }

  public void intakeStop() {
    IntakeExtendMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean isIntakeUnlocked(){
    // if(IntakePistonLock.get().equals(DoubleSolenoid.Value.kForward)){
    //   return true;
    // } else{
    //   return false;
    // }
    return intakeOut;
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Encoder Val", IntakeExtendMotor.getSelectedSensorPosition());
  }
}
