package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopingArmSubsystem extends SubsystemBase {

    public static TalonFX tm; //telescoping motor
    
    public static PIDController PID;
    public static double currentLength; 
    
    // Initialize here
    static {
        // motor stuff
        tm = new TalonFX(Constants.telscopingMotorPort);

        //pid stuff
        PID = new PIDController(Constants.tP, Constants.tI, Constants.tD);
    }

    public static void setArmLength(double setpointLength) {
        //must calculate current length
        currentLength = Constants.armLengths[0] + ( tm.getSelectedSensorPosition() * Constants.ticksToFeet );    
        tm.set(ControlMode.PercentOutput, PID.calculate(currentLength, setpointLength));
    }
    
    public static void setSpeed(double speed) {
        tm.set(ControlMode.PercentOutput, speed);
    }

    public static void resetTelescopingEncoder() {
        
    }
}