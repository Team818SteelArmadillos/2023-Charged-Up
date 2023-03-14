package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

public class TelescopingArmSubsystem extends SubsystemBase {

    public TalonFX telescopingMotor; //telescoping motor
    
    public double currentLength;
    //public DutyCycleEncoder encoder;

    // Initialize herems
    public TelescopingArmSubsystem() {
        // motor stuff
        telescopingMotor = new TalonFX(Constants.telscopingMotorPort);

        //pid stuff
        configureMotor();
        //grabs rotations of motor
        /*
        encoder = new DutyCycleEncoder(Constants.boreEncoder);
        encoder.reset();
        encoder.setDistancePerRotation(1);
        SmartDashboard.putNumber("Telescoping Arm Rotations", encoder.getDistance());
        */
    }

    public void setArmLength(double setpointLength) {
        telescopingMotor.set(ControlMode.Position, setpointLength);
    }
    
    public void setSpeed(double speed) {
        telescopingMotor.set(ControlMode.PercentOutput, speed);
    }

    public void resetEncoder() {
        telescopingMotor.setSelectedSensorPosition(0);
    }

    public double getEncoder() {
        return telescopingMotor.getSelectedSensorPosition();
    }

    public void configureMotor() {
        telescopingMotor.setInverted(true);
        telescopingMotor.config_kP(0, Constants.tP);
        telescopingMotor.config_kI(0, Constants.tI);
        telescopingMotor.config_kD(0, Constants.tD);
        telescopingMotor.configAllowableClosedloopError(0, 300);
    }
}