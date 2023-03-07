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
        bikeBreak = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[6], Constants.pneumaticPorts[7]);
    }

    public static void setArmLocked() {
        bikeBreak.set(Value.kForward);
    }

    public static void setArmUnlocked() {
        bikeBreak.set(Value.kReverse);
    }

    public static boolean isOpen() {
        if (bikeBreak.get() == DoubleSolenoid.Value.kReverse) {
            return true;
        } else {
            return false;
        }
    }

}
