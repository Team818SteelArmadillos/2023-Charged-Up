package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PivotingArmSubsystem extends SubsystemBase {
    public TalonSRX pm1; // pivoting motor 1
    public TalonSRX pm2;
    public TalonSRX pm3;
    
    public PIDController PID;
    public DutyCycleEncoder encoder; // = new Encoder(0, 0, 0); // LETS CODE THIS THING!!
    public double currentAngle; 
    
    // Initialize here
    public PivotingArmSubsystem() {
        // motor stuff
        pm1 = new TalonSRX(Constants.pivotingMotorPorts[0]);
        pm2 = new TalonSRX(Constants.pivotingMotorPorts[1]);
        pm3 = new TalonSRX(Constants.pivotingMotorPorts[2]);

        pm2.follow(pm1); 
        pm3.follow(pm1); //makes motors 2 and 3 follow 1 so that only 1 needs to be set

        //pid stuff
        PID = new PIDController(Constants.pP, Constants.pivotI, Constants.pivotD);
        
        //encoder stuff
        encoder = new DutyCycleEncoder(Constants.THROUGH_BORE_ENCODER); //these encoder paramters are undefined since
        encoder.reset();
        SmartDashboard.putNumber( "Pivoting Arm Encoder", encoder.get() );
    }

    public void setPivotAngle(double setpointAngle) {
        currentAngle = encoder.get() * 360; // Converts the encoder rotation values to degrees
        pm1.set(ControlMode.PercentOutput, PID.calculate(currentAngle, setpointAngle));
    }

    public void setPivotSpeed(double pivotSpeed) {
        pm1.set(ControlMode.PercentOutput, pivotSpeed);
        SmartDashboard.putNumber("pivotingEncoder", encoder.get());
    }

    public void resetPivotingEncoder() {
        encoder.reset();
    }

}