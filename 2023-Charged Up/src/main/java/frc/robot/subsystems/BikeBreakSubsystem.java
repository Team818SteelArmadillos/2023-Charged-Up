package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class BikeBreakSubsystem extends SubsystemBase {
    
     DoubleSolenoid bikeBreak;

    // Initialize here
    public BikeBreakSubsystem() {
        bikeBreak = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[6], Constants.pneumaticPorts[7]);
        setArmLocked();
    }

    public void setArmLocked() {
        bikeBreak.set(Value.kForward);
    }

    public void setArmUnlocked() {
        bikeBreak.set(Value.kReverse);
    }

    public void toggle() {
        bikeBreak.toggle();
    }

    public boolean isOpen() {
        return bikeBreak.get().equals(Value.kReverse);
    }

}
