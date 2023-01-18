package com.team818.frc2023.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {

    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;
    public static TalonFXConfiguration feederFXConfig;
    public static TalonFXConfiguration shooterFXConfig;
    public static TalonFXConfiguration climberFXConfig;
    public static TalonFXConfiguration intakeFXConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();
        feederFXConfig = new TalonFXConfiguration();
        shooterFXConfig = new TalonFXConfiguration();
        climberFXConfig = new TalonFXConfiguration();
        intakeFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration();

        
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration();
           
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Feeder Falcon Configuration */
        SupplyCurrentLimitConfiguration feederSupplyLimit = new SupplyCurrentLimitConfiguration(

        ); 


        feederFXConfig.supplyCurrLimit = feederSupplyLimit;
        feederFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Shooter Falcon Configuration */
        SupplyCurrentLimitConfiguration shooterSupplyLimit = new SupplyCurrentLimitConfiguration(

        );


        shooterFXConfig.supplyCurrLimit = shooterSupplyLimit;


        /* Climber Falcon Configuration */
        SupplyCurrentLimitConfiguration climberSupplyLimit = new SupplyCurrentLimitConfiguration(
            
        );

        
        climberFXConfig.supplyCurrLimit = climberSupplyLimit;
        climberFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        

        /* Intake Falcon Configuration */
        SupplyCurrentLimitConfiguration intakeSupplyLimit = new SupplyCurrentLimitConfiguration(
            
        );

        /*intakeFXConfig.slot0.kP = 0.01;
        intakeFXConfig.slot0.kI = 0.0;
        intakeFXConfig.slot0.kD = 0.01;
        intakeFXConfig.slot0.kF = 100; */

        intakeFXConfig.supplyCurrLimit = intakeSupplyLimit;

        
        






        

    }

}