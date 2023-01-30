package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class BikeBreakSubsystem extends SubsystemBase {
    
    static DoubleSolenoid bikeBreak;

    // Initialize here
    static {
        bikeBreak = new DoubleSolenoid(Constants.clawPistonPort[1], PneumaticsModuleType.CTREPCM, Constants.clawPistonPort[0], Constants.clawPistonPort[2]);
    }

    public static void setIntakeForward() {
        bikeBreak.set(DoubleSolenoid.Value.kForward);
    }

    public static void setInkateReverse() {
        bikeBreak.set(DoubleSolenoid.Value.kReverse);
    }

}
