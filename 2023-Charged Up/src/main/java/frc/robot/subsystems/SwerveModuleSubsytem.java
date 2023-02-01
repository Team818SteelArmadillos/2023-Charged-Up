// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;



public class SwerveModuleSubsytem extends SubsystemBase {
  //private TalonFX frontleftdrive, frontrightdrive, backleftdrive, backrightdrive,  frontleftturn, frontrightturn, backleftturn, backrightturn;
  public SwerveModule frontleftSM, frontrightSM, backleftSM, backrightSM;

  public static class SwerveModule {
    public int m_moduleNumber;
    private TalonSRX m_drivemotor;
    private TalonSRX m_turningmotor;
    private CANCoder m_cancoder;
    private Encoder m_driveEncoder;
    private double m_lastAngle;
    private double m_offset;
    private boolean m_turningInverted;
    private boolean m_driveInverted;
    private boolean m_cancoderInverted;
    public static SwerveModuleState desiredState;
    

    public SwerveModule(int moduleNumber, int drivemotor,   int turningmotor, int cancoder, double lastAngle, 
      double offset, boolean turningInverted, boolean driveInverted, boolean cancoderInverted, int ChannelA, int ChannelB){
      m_moduleNumber = moduleNumber;
      m_drivemotor = new TalonSRX(drivemotor);
      m_turningmotor = new TalonSRX(turningmotor);
      m_cancoder = new CANCoder(cancoder);
      m_driveEncoder = new Encoder(ChannelA, ChannelB);
      m_lastAngle = lastAngle; //change to get state in order to get angle
      m_offset = offset;
      m_turningInverted = turningInverted;
      m_driveInverted = driveInverted;
      m_cancoderInverted = cancoderInverted;

      configCanCoder();
      configDriveMotor();
      configTurningMotor();

      
      
    }

    public void setDesiredState(Rotation2d angle, double speed, boolean openLoop ){

      desiredState = new SwerveModuleState(speed, angle);

      desiredState = CTREModuleState.optimize(desiredState, getState().angle);

      if(openLoop){
        m_drivemotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond/1);//replace 1 with the max speed
      } else{
        m_drivemotor.set(ControlMode.Velocity, Constants.mpsToFalcon(desiredState.speedMetersPerSecond, 4, 6.67),
         DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
      }

     double newAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (0)) ? m_lastAngle : angle.getDegrees();
     m_turningmotor.set(ControlMode.Position, DriveConstants.degreesToFalcon(newAngle));
     
     m_lastAngle = newAngle;
    }

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0); //put the robot characteristics from sysID here

    private void resetToAbsolute() {
      m_turningmotor.setSelectedSensorPosition(DriveConstants.degreesToFalcon(getCanCoder().getDegrees()));
    }

    public static SwerveModuleState getDesiredState(){
      return desiredState;
    }
    private Rotation2d getCanCoder() {
      return Rotation2d.fromDegrees(m_cancoder.getAbsolutePosition());
    }

    private void configCanCoder(){
      m_cancoder.configFactoryDefault();
      m_cancoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
      m_cancoder.configSensorDirection(m_cancoderInverted);
    }

    private void configTurningMotor() {
      m_turningmotor.configFactoryDefault();
      //m_turningmotor.configAllSettings(TalonSRXConfigurationon);
      m_turningmotor.setInverted(m_turningInverted);
      m_turningmotor.setNeutralMode(NeutralMode.Coast);
      resetToAbsolute();
    }

    private void configDriveMotor() {
      m_drivemotor.configFactoryDefault();
      //m_drivemotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
      m_drivemotor.setInverted(m_driveInverted);
      m_drivemotor.setNeutralMode(NeutralMode.Brake);
      resetToAbsolute();
    }
    public double getDriveEncoder(){
      return m_driveEncoder.getRaw();
    }
    

    public SwerveModuleState getState(){
      double velocity = Constants.falconToMPS(m_driveEncoder.getRaw(), 4, 1); //replace 0 to circumfrence, and 1 with gear ratio
      Rotation2d angle = Rotation2d.fromDegrees(DriveConstants.falconToDegrees(m_turningmotor.getSelectedSensorPosition()));//get gear ratio from turning motor
      return new SwerveModuleState(velocity, angle);
    }

    public void zeroModule(){
      m_turningmotor.setSelectedSensorPosition(DriveConstants.degreesToFalcon(m_offset));//get gear ratio for turning motor
    }
    public void moduleNumber(){
      return ;
    }
  }
  /** Creates a new SwerveModuleSubsytem. */
  public SwerveModuleSubsytem() {
    //frontleftdrive = new TalonFX()
    frontleftSM = new SwerveModule(0, 5, 1, 1, 0, 0, true, true, true, 0, 1);
    frontrightSM = new SwerveModule(1, 4, 10, 10, 0, 0, true, true, true, 2, 3);
    backleftSM = new SwerveModule(2, 6, 2, 2, 0, 0, true, true, true, 4, 5);
    backrightSM = new SwerveModule(3, 7, 3, 3, 0, 0, true, true, true, 6, 7);
    
  }

  



  private final PIDController m_turingPIDController = 
    new PIDController(0, 0, 0);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    
  }
}
