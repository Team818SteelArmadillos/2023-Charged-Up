package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PistonClawSubsystem extends SubsystemBase {
    
     DoubleSolenoid pistonClaw;

    // Initialize here
    public PistonClawSubsystem() {
        pistonClaw = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[5], Constants.pneumaticPorts[4]);
        pistonClaw.set(DoubleSolenoid.Value.kReverse);
    }

    public void setClawClosed() {
        pistonClaw.set(DoubleSolenoid.Value.kForward);
    }

    public void setClawOpen() {
        pistonClaw.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggle() {
        pistonClaw.toggle();
    }

    public boolean isOpen() {
        if (pistonClaw.get().equals(DoubleSolenoid.Value.kForward)) {
            return true;
        } else {
            return false;
        }
    }

}
