package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotingArmSubsystem extends SubsystemBase {
    public static TalonSRX pivotingMotorOne;
    public static VictorSPX pivotingMotorTwo;
    public static VictorSPX pivotingMotorThree;
    
    // Initialize here
    static {
    }


}