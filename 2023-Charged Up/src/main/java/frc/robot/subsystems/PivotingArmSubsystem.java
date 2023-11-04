package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PivotingArmSubsystem extends SubsystemBase {
    public TalonSRX pm1; // pivoting motor 1
    public TalonSRX pm2;
    public TalonSRX pm3;
    
    public PIDController PID;
    public ArmFeedforward armFeedForward;
    public static DutyCycleEncoder encoder; // = new Encoder(0, 0, 0); // LETS CODE THIS THING!!
    //public double currentAngle; 
    public int armCounts;
    
    public DoubleSolenoid bikeBreak;
    
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
        armFeedForward = new ArmFeedforward(armCounts, armCounts, armCounts);


        //encoder stuff
        encoder = new DutyCycleEncoder(Constants.THROUGH_BORE_ENCODER); //these encoder paramters are undefined since
        //encoder.reset();
        SmartDashboard.putNumber( "Pivoting Arm Encoder", encoder.get() );

        bikeBreak = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[6], Constants.pneumaticPorts[7]);
        setArmLocked();

        armCounts = 0;
        
    }

    public boolean onSetPoint(){
        return PID.atSetpoint();
    }

    public void setPivotAngle(double setpointAngle) {

        //Every time setPivotAngle is called, it must perform three functions.
        //The first of which is to calculate the pidOutput based on the previously defined PID controller, given the current angle and desired angle
        //The second function acts as a counter to see how long the arm has been within tolerance of the setpoint.
        //Simply put, the third function checks if the arm has been at the setpoint for "long enough." If so, it locks the arm. Otherwise, it keeps it 
        //unlocked and keeps moving toward its desired setpoint.

        double pidOutput = PID.calculate(getAngle(), setpointAngle);

        if ( PID.atSetpoint() )  {
            armCounts++;
        } else {
            armCounts = 0;
        }
        
        if (armCounts > Constants.armSetpointCounter) { 
            setArmLocked(); 
        } else {
            setArmUnlocked();
            setPivotSpeed(pidOutput);
        }

    }

    public void setPivotSpeed(double pivotSpeed) {
        pm1.set(ControlMode.PercentOutput, pivotSpeed);
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public double getEncoder() {
        return encoder.get() - Constants.encoderOvershoot;
    }

    public double getAngle() {
        return (getEncoder() * 360); //- Constants.encoderOvershoot;
    }

    public void setArmUnlocked() {
        bikeBreak.set(Value.kForward);
    }

    public void setArmLocked() {
        bikeBreak.set(Value.kReverse);
    }

    public void toggle() {
        bikeBreak.toggle();
    }

    public boolean isBikeBreakEngaged() {
        return bikeBreak.get().equals(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void periodic(){ 
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber( "Pivoting Arm Encoder", encoder.get() );
    }
}