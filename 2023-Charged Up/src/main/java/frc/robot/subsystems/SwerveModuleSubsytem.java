// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SwerveModuleSubsytem extends SubsystemBase {
  //private TalonFX frontleftdrive, frontrightdrive, backleftdrive, backrightdrive,  frontleftturn, frontrightturn, backleftturn, backrightturn;
  private SwerveModule frontleftSM, frontrightSM, backleftSM, backrightSM;

  public class SwerveModule {
    private TalonFX m_drivemotor;
    private TalonFX m_turningmotor;
    private double m_lastAngle;
    private double m_offset;
    

    public SwerveModule(int drivemotor, int turningmotor){
      m_drivemotor = new TalonFX(drivemotor);
      m_turningmotor = new TalonFX(turningmotor);

    }
  }
  /** Creates a new SwerveModuleSubsytem. */
  public SwerveModuleSubsytem() {
    //frontleftdrive = new TalonFX()
    frontleftSM = new SwerveModule(1, 2);
    frontrightSM = new SwerveModule(3, 4);
    backleftSM = new SwerveModule(5, 6);
    backrightSM = new SwerveModule(7, 8);
    
  }

  private final PIDController m_turingPIDController =
    new PIDController(0, 0, 0);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
