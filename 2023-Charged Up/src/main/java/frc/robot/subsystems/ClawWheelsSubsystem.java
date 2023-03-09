package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawWheelsSubsystem extends SubsystemBase {
    
    public static CANSparkMax cwm1; //Claw wheel motor 1
    public static CANSparkMax cwm2; 
    
    // Initialize here
    static {
        // motor stuff
        cwm1 = new CANSparkMax(Constants.clawWheelMotorPort[0], MotorType.kBrushless);
        cwm2 = new CANSparkMax(Constants.clawWheelMotorPort[1], MotorType.kBrushless);
        
        cwm1.setSmartCurrentLimit(Constants.neoAmpLimit);
        cwm2.setSmartCurrentLimit(Constants.neoAmpLimit);
        
        
        cwm2.setInverted(true);
    }

    
    public static void setIntakeSpeed(double intakeSpeed) {
        //must calculate current length
        double _intakeSpeed;

        _intakeSpeed = 0.2 * (intakeSpeed + 1);

        cwm1.set(_intakeSpeed);
        cwm2.set(_intakeSpeed);
        SmartDashboard.putNumber("_IntakeSpeed", _intakeSpeed);
        
    }
    
}
