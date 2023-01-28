package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {

    public TalonSRXConfiguration swerveAngleSRXConfig;
    public TalonSRXConfiguration swerveDriveSRXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;
    public static TalonFXConfiguration feederFXConfig;
    public static TalonFXConfiguration shooterFXConfig;
    public static TalonFXConfiguration climberFXConfig;
    public static TalonFXConfiguration intakeFXConfig;


    public CTREConfigs(){
        swerveAngleSRXConfig = new TalonSRXConfiguration();
        swerveDriveSRXConfig = new TalonSRXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();
        feederFXConfig = new TalonFXConfiguration();
        shooterFXConfig = new TalonFXConfiguration();
        climberFXConfig = new TalonFXConfiguration();
        intakeFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
        TalonSRXConfiguration angleSupplyLimit = new TalonSRXConfiguration();

        
        //swerveAngleSRXConfig.continuousCurrentLimit = angleSupplyLimit;
        //swerveAngleSRXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration();
           
        //swerveDriveSRXConfig.supplyCurrLimit = driveSupplyLimit;
        //swerveDriveSRXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Feeder Falcon Configuration */
        SupplyCurrentLimitConfiguration feederSupplyLimit = new SupplyCurrentLimitConfiguration(

        ); 


        
        






        

    }

}