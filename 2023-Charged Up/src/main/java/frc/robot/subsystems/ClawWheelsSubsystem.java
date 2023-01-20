package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawWheelsSubsystem extends SubsystemBase {

    public static VictorSPX cwm1; //Claw wheel motor 1
    public static VictorSPX cwm2; 
    
    // Initialize here
    static {
        // motor stuff
        cwm1 = new VictorSPX(Constants.clawWheelMotorPort[0]);
        cwm2 = new VictorSPX(Constants.clawWheelMotorPort[1]);

        cwm2.setInverted(true);
        cwm2.follow(cwm1);
    }

    public static void setIntakeSpeed(double intakeSpeed) {
        //must calculate current length
        cwm1.set(ControlMode.PercentOutput, intakeSpeed);
    }
    
}
