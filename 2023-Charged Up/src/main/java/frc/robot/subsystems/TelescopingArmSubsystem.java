package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopingArmSubsystem extends SubsystemBase {

    public static TalonSRX tm; //telescoping motor
    
    public static PIDController PID;
    public static Encoder encoder; // = new Encoder(0, 0, 0); // LETS CODE THIS THING!!
    public static double currentLength; 
    
    // Initialize here
    static {
        // motor stuff
        tm = new TalonSRX(Constants.telscopingMotorPort);

        //pid stuff
        PID = new PIDController(Constants.tP, Constants.tI, Constants.tD);
        
        //encoder stuff
        encoder = new Encoder(0, 0, 0);
        encoder.reset();

    }

    public static void setArmLength(double setpointLength) {
        //must calculate current length
        currentLength = Constants.minArmLength + ( encoder.getRaw() * Constants.ticksToFeet );    
        tm.set(ControlMode.PercentOutput, PID.calculate(currentLength, setpointLength));
    }
    
}
