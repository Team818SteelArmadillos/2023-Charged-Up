package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PistonClawSubsystem extends SubsystemBase {
    
    static DoubleSolenoid pistonClaw;

    // Initialize here
    static {
        pistonClaw = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[5], Constants.pneumaticPorts[4]);
    }

    public static void setClawClosed() {
        pistonClaw.set(DoubleSolenoid.Value.kForward);
    }

    public static void setClawOpen() {
        pistonClaw.set(DoubleSolenoid.Value.kReverse);
    }

    public static boolean isOpen() {
        if (pistonClaw.get() == DoubleSolenoid.Value.kReverse) {
            return true;
        } else {
            return false;
        }
    }

}
