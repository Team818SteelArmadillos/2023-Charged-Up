package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PistonClawSubsystem extends SubsystemBase {
    
    static DoubleSolenoid pistonClaw;

    // Initialize here
    static {
        pistonClaw = new DoubleSolenoid(Constants.clawPistonPort[1], PneumaticsModuleType.CTREPCM, Constants.clawPistonPort[0], Constants.clawPistonPort[2]);
    }

    public static void setClawClosed() {
        pistonClaw.set(DoubleSolenoid.Value.kForward);
    }

    public static void setClawOpen() {
        pistonClaw.set(DoubleSolenoid.Value.kReverse);
    }

    public static boolean isOpen() {
        if (pistonClaw.get() == DoubleSolenoid.Value.kForward) {
            return true;
        } else {
            return false;
        }
    }

}
