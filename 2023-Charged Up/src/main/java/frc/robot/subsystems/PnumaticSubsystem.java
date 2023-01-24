package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.oi;


public class PnumaticSubsystem extends SubsystemBase {
    public static Compressor compressor = new Compressor(null);
    public static DoubleSolenoid solenoid = new DoubleSolenoid(null, 0, 1);
        // initializin' here
        static {
            if (oi.getOperator().getLeftBumper()) { //gets input from the operator bumpers
                solenoid.set(DoubleSolenoid.Value.kForward); //changes voltage in the solenoid to make the hand close
            }
            else if (oi.getOperator().getRightBumper()) { //may have to edit for right voltages teehee IDK
                solenoid.set(DoubleSolenoid.Value.kReverse);
            }

        }
}