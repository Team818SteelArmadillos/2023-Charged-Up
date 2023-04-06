package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
    public TalonSRX pm1; // pivoting motor 1
    public TalonSRX pm2;
    public TalonSRX pm3;
    
    public PIDController PivotingPID;
    public static DutyCycleEncoder encoder; // = new Encoder(0, 0, 0); // LETS CODE THIS THING!! 
    public int armCounts;
    
    public TalonFX telescopingMotor;
    
    public double currentLength;
    
    public DoubleSolenoid bikeBreak;

    // Initialize here
    public ArmSubsystem() {
        // motor stuff
        pm1 = new TalonSRX(Constants.pivotingMotorPorts[0]);
        pm2 = new TalonSRX(Constants.pivotingMotorPorts[1]);
        pm3 = new TalonSRX(Constants.pivotingMotorPorts[2]);

        pm2.follow(pm1); 
        pm3.follow(pm1); //makes motors 2 and 3 follow 1 so that only 1 needs to be set
        
        telescopingMotor = new TalonFX(Constants.telscopingMotorPort);
        configureMotor();
        
        //pid stuff
        PivotingPID = new PIDController(Constants.pP, Constants.pivotI, Constants.pivotD);
        PivotingPID.setTolerance(Constants.pPIDTolerance);
        PivotingPID.reset();
        
        //encoder stuff
        encoder = new DutyCycleEncoder(Constants.THROUGH_BORE_ENCODER); //these encoder paramters are undefined since
        //encoder.reset();

        bikeBreak = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[6], Constants.pneumaticPorts[7]);
        setArmLocked();

        armCounts = 0;
    }

     /*==============================
            pivoting stuff
    ==============================*/

    public boolean onPivotingSetPoint() {
        return PivotingPID.atSetpoint();
    }

    public void setPivotAngle(double setpointAngle) {

        //Every time setPivotAngle is called, it must perform three functions.
        //The first of which is to calculate the pidOutput based on the previously defined PID controller, given the current angle and desired angle
        //The second function acts as a counter to see how long the arm has been within tolerance of the setpoint.
        //Simply put, the third function checks if the arm has been at the setpoint for "long enough." If so, it locks the arm. Otherwise, it keeps it 
        //unlocked and keeps moving toward its desired setpoint.

        double pidOutput = PivotingPID.calculate(getPivotAngle(), setpointAngle);

        if ( PivotingPID.atSetpoint() )  {
            armCounts++;
        } else {
            armCounts = 0;
        }
        
        if (armCounts >= Constants.armSetpointCounter) { 
            setArmLocked();
            setPivotSpeed(0.0);
            armCounts = Constants.armSetpointCounter; // prevent overflow
        } else {
            setArmUnlocked();
            setPivotSpeed(pidOutput);
        }

    }

    public void setPivotSpeed(double pivotSpeed) {
        pm1.set(ControlMode.PercentOutput, pivotSpeed);
    }

    // public void resetPivotingEncoder() {
    //     encoder.reset();
    // }

    public double getPivotingEncoder() {
        return encoder.get() - Constants.encoderOvershoot;
    }

    public double getPivotAngle() {
        return (getPivotingEncoder() * 360); //- Constants.encoderOvershoot;
    }

    /*==============================
            telescoping stuff
    ==============================*/

    public boolean getBottomLimitswitch() {
        return telescopingMotor.isRevLimitSwitchClosed() == 1;
    }

    public boolean getTopLimitswitch() {
        return telescopingMotor.isFwdLimitSwitchClosed() == 1;
    }

    public void setArmLength(double setpointLength) {
        telescopingMotor.set(ControlMode.Position, setpointLength);
    }
    
    public void setSpeed(double speed) {
        telescopingMotor.set(ControlMode.PercentOutput, speed);
    }

    public void resetTelescopingEncoder() {
        telescopingMotor.setSelectedSensorPosition(0);
    }

    public double getTelescopingEncoder() {
        return telescopingMotor.getSelectedSensorPosition();
    }

    public boolean onSetPoint(double positon) {
        return Math.abs(telescopingMotor.getSelectedSensorPosition() - positon) < Constants.tTolerance;
    }

    public void configureMotor() {
        telescopingMotor.setInverted(true);
        telescopingMotor.config_kP(0, Constants.tP);
        telescopingMotor.config_kI(0, Constants.tI);
        telescopingMotor.config_kD(0, Constants.tD);
        telescopingMotor.configAllowableClosedloopError(0, Constants.tTolerance);
        telescopingMotor.setSelectedSensorPosition(0);
    }

     /*==============================
            bike break stuff
    ==============================*/

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
        if (bikeBreak.get().equals(DoubleSolenoid.Value.kReverse)) {
            return true;
        } else {
            return false;
        }
    }

    //PERIODIC

    @Override
    public void periodic(){ 
        SmartDashboard.putNumber("Arm Angle", getPivotAngle());
        SmartDashboard.putNumber("Pivoting Arm Encoder RAW", encoder.get());
        SmartDashboard.putNumber("Telescoping Arm Encoder", getTelescopingEncoder());
    }

}