package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawWheelsSubsystem extends SubsystemBase {
    
    public CANSparkMax cwm1; //Claw wheel motor 1
    public CANSparkMax cwm2; 
    
    // Initialize here
    public ClawWheelsSubsystem() {
        // motor stuff
        cwm1 = new CANSparkMax(Constants.clawWheelMotorPort[0], MotorType.kBrushless);
        cwm2 = new CANSparkMax(Constants.clawWheelMotorPort[1], MotorType.kBrushless);
        
        cwm1.setSmartCurrentLimit(Constants.neoAmpLimit);
        cwm2.setSmartCurrentLimit(Constants.neoAmpLimit);
        
        
        cwm2.setInverted(true);
    }

    
    public void setIntakeSpeed(double intakeSpeed) {
        cwm1.set(intakeSpeed);
        cwm2.set(intakeSpeed);
        SmartDashboard.putNumber("_IntakeSpeed", intakeSpeed);
        
    }
    
}
