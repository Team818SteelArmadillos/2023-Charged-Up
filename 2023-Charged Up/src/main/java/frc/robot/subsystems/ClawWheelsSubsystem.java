package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
        cwm2.follow(cwm1);
    }

    
    public static void setIntakeSpeed(double intakeSpeed) {
        //must calculate current length
        cwm1.set(intakeSpeed);
    }
    
}
