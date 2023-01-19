package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PivotingArmSubsystem extends SubsystemBase {
    public static TalonSRX  pm1; // pivoting motor 1
    public static VictorSPX pm2;
    public static VictorSPX pm3;
    
    public static PIDController PID;
    public static Encoder encoder; // = new Encoder(0, 0, 0); // LETS CODE THIS THING!!
    public static double currentAngle; 
    
    // Initialize here
    static {
        // motor stuff
        pm1 = new TalonSRX(Constants.pivotingMotorPorts[0]);
        pm2 = new VictorSPX(Constants.pivotingMotorPorts[1]);
        pm3 = new VictorSPX(Constants.pivotingMotorPorts[2]);

        pm2.follow(pm1); 
        pm3.follow(pm1); //makes motors 2 and 3 follow 1 so that only 1 needs to be set

        //pid stuff
        PID = new PIDController(Constants.pP, Constants.pivotI, Constants.pivotD);
        
        //encoder stuff
        encoder = new Encoder(0, 0, 0); //these encoder paramters are undefined since
        encoder.reset();

    }

    public static void setPivotAngle(double setpointAngle) {
        currentAngle = encoder.get() * Constants.ticksToDegrees; // Converts the ticks from encoder to degrees
        pm1.set(ControlMode.PercentOutput, PID.calculate(currentAngle, setpointAngle));
    }

}