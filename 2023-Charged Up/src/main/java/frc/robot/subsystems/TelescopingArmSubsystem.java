package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopingArmSubsystem extends SubsystemBase {

    public TalonFX tm; //telescoping motor
    
    public PIDController PID;
    public double currentLength; 
    public DutyCycleEncoder encoder;

    // Initialize herems
    public TelescopingArmSubsystem() {
        // motor stuff
        tm = new TalonFX(Constants.telscopingMotorPort);

        //pid stuff
        PID = new PIDController(Constants.tP, Constants.tI, Constants.tD);

        //grabs rotations of motor
        encoder = new DutyCycleEncoder(Constants.boreEncoder);
        encoder.reset();
        encoder.setDistancePerRotation(1);
        SmartDashboard.putNumber("Telescoping Arm Rotations", encoder.getDistance());
    }

    public void setArmLength(double setpointLength) {
        //must calculate current length
        currentLength = Constants.armLengths[0] + ( tm.getSelectedSensorPosition() * Constants.ticksToFeet );    
        tm.set(ControlMode.PercentOutput, PID.calculate(currentLength, setpointLength));
    }
    
    public void setSpeed(double speed) {
        tm.set(ControlMode.PercentOutput, speed);
    }

    public void resetTelescopingEncoder() {
        
    }
}