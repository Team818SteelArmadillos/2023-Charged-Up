// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;



public class SwerveModuleSubsytem extends SubsystemBase {
  //private TalonFX frontleftdrive, frontrightdrive, backleftdrive, backrightdrive,  frontleftturn, frontrightturn, backleftturn, backrightturn;
  private SwerveModule frontleftSM, frontrightSM, backleftSM, backrightSM;

  public class SwerveModule {
    public int m_moduleNumber;
    private TalonFX m_drivemotor;
    private TalonFX m_turningmotor;
    private CANCoder m_cancoder;
    private double m_lastAngle;
    private double m_offset;
    private boolean m_turningInverted;
    private boolean m_driveInverted;
    

    public SwerveModule(int moduleNumber, int drivemotor,   int turningmotor, int cancoder, double lastAngle, 
      double offset, boolean turningInverted, boolean driveInverted){
      m_moduleNumber = moduleNumber;
      m_drivemotor = new TalonFX(drivemotor);
      m_turningmotor = new TalonFX(turningmotor);
      m_cancoder = new CANCoder(cancoder);
      m_lastAngle = lastAngle; //change to get state in order to get angle
      m_offset = offset;
      m_turningInverted = turningInverted;
      m_driveInverted = driveInverted;

      

    }

    public void setDesiredState(SwerveModule desiredState, boolean openLoop){
      //this is used to optimize the time to get to get to desired angle
    }

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 2); //put the robot characteristics from sysID here

    private void resetToAbsolute() {
      m_turningmotor.setSelectedSensorPosition(Constants.degreesToFalcon(getCanCoder().getDegrees(), 2048));
    }

    private Rotation2d getCanCoder() {
      return Rotation2d.fromDegrees(m_cancoder.getAbsolutePosition());
    }
  }
  /** Creates a new SwerveModuleSubsytem. */
  public SwerveModuleSubsytem() {
    //frontleftdrive = new TalonFX()
    frontleftSM = new SwerveModule(0, 1, 2, 3, 0, 0, true, true);
    frontrightSM = new SwerveModule(0, 3, 4, 5, 0, 0, true, true);
    backleftSM = new SwerveModule(0, 5, 6, 7, 0, 0, true, true);
    backrightSM = new SwerveModule(0, 7, 8, 9, 0, 0, true, true);
    
  }

  



  private final PIDController m_turingPIDController =
    new PIDController(0, 0, 0);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private void configTurningMotor(){
    
    
  }
}
