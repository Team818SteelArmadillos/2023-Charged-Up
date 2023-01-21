package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PnumaticSubsystem extends SubsystemBase {
    public static Compressor compressor = new Compressor();
    public static DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);
        // initializin' here
        static {
            if (OI.getOperator().getBumper(Hand.kLeft)) { //gets input from the operator bumpers
                solenoid.set(DoubleSolenoid.Value.kForward); //changes voltage in the solenoid to make the hand close
            }
            else if (OI.getOperator().getBumper(Hand.kRight)) { //may have to edit for right voltages teehee IDK
                solenoid.set(DoubleSolenoid.Value.kReverse);
            }

        }
}