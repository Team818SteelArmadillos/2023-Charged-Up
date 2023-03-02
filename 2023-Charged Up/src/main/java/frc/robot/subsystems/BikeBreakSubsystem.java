package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class BikeBreakSubsystem extends SubsystemBase {
    
    static DoubleSolenoid bikeBreak;

    // Initialize here
    static {
        bikeBreak = new DoubleSolenoid(Constants.clawPistonPort[0], PneumaticsModuleType.CTREPCM, Constants.clawPistonPort[1], Constants.clawPistonPort[2]);
    }

    public static void setArmLocked() {
        bikeBreak.set(Value.kForward);
    }

    public static void setArmUnlocked() {
        bikeBreak.set(Value.kReverse);
    }

}
