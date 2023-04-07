package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawSubsystem extends SubsystemBase {

    private boolean inCubeMode;
    
     DoubleSolenoid pistonClaw;
     
    public CANSparkMax cwm1; //Claw wheel motor 1
    public CANSparkMax cwm2; 

    // Initialize here
    public ClawSubsystem() {
        pistonClaw = new DoubleSolenoid(Constants.pneumaticPistonPort, PneumaticsModuleType.CTREPCM, Constants.pneumaticPorts[5], Constants.pneumaticPorts[4]);
        pistonClaw.set(DoubleSolenoid.Value.kReverse);
        inCubeMode = false;

        
        // motor stuff
        cwm1 = new CANSparkMax(Constants.clawWheelMotorPort[0], MotorType.kBrushless);
        cwm2 = new CANSparkMax(Constants.clawWheelMotorPort[1], MotorType.kBrushless);
        
        cwm1.setSmartCurrentLimit(Constants.neoAmpLimit);
        cwm2.setSmartCurrentLimit(Constants.neoAmpLimit);
        
        cwm2.setInverted(true);
    }

    public void setClawClosed() {
        pistonClaw.set(DoubleSolenoid.Value.kReverse);
        inCubeMode = false;
    }

    public void setClawOpen() {
        pistonClaw.set(DoubleSolenoid.Value.kForward);
        inCubeMode = true;
    }

    public void toggle() {
        pistonClaw.toggle();
        inCubeMode = !inCubeMode;
    }

    public boolean inCubeMode() {
        return inCubeMode;
    }
    
    public void setIntakeSpeed(double intakeSpeed) {
        cwm1.set(intakeSpeed);
        cwm2.set(intakeSpeed);
    }

    public double getMotorCurrent() {
        return (cwm1.getOutputCurrent() + cwm2.getOutputCurrent()) / 2.0;
    }

    public double getMotorVoltage() {
        return (cwm1.getBusVoltage() + cwm2.getBusVoltage()) / 2.0;
    }

    public boolean isOpen() {
        if (pistonClaw.get().equals(DoubleSolenoid.Value.kForward)) {
            return true;
        } else {
            return false;
        }
    }
    
    // @Override
    // public void periodic() {
    //     SmartDashboard.putNumber("Claw Current", getMotorCurrent());
    //     SmartDashboard.putNumber("Claw Voltage", getMotorVoltage());

    //     if (isOpen()) {
    //         SmartDashboard.putString("Claw Mode", "OPEN / CUBE");
    //     } else {
    //         SmartDashboard.putString("Claw Mode", "CLOSED / CONE");
    //     }
    // }

}
