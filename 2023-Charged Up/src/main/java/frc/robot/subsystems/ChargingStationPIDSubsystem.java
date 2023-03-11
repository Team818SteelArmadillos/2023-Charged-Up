package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


//im desperately trying to figure out how to PID lol - casey
//if you can add anything or help at all PLEASE DO
//I'm gonna do some stuff... - Aiden

//goober


public class ChargingStationPIDSubsystem extends SubsystemBase {

    public Pigeon2 pigeon;
    public double currentPitch;
    public double goalPitch = 0;
    public PIDController pid;
    public TalonFX dm; //placeholder for driving motors

    //initialize here! 😂
    public ChargingStationPIDSubsystem() {
        SmartDashboard.putNumber("Pitch", pigeon.getPitch());
        currentPitch = pigeon.getPitch();
        pid = new PIDController(Constants.csP, Constants.csI, Constants.csD);
        pid.setTolerance(Constants.csTolerance);
        pid.reset();
    }

    public void fixPitch() {
        dm.set(ControlMode.PercentOutput, pid.calculate(currentPitch, goalPitch));
    }

}
package frc.robot.subsystems;

//im desperately trying to figure out how to PID lol - casey
//if you can add anything or help at all PLEASE DO

//goober


/*
public class ChargingStationPID extends SubsystemBase {

    static final double kOffBalanceAngleThresholdDegrees = 10;
    static final double kOonBalanceAngleThresholdDegrees  = 5;

    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {

            double xAxisRate            = stick.getX();
            double yAxisRate            = stick.getY();
            double pitchAngleDegrees    = ahrs.getPitch();
            double rollAngleDegrees     = ahrs.getRoll();
            
            if ( !autoBalanceXMode && 
                (Math.abs(pitchAngleDegrees) >= 
                Math.abs(kOffBalanceAngleThresholdDegrees))) {
                autoBalanceXMode = true;
            }
            else if ( autoBalanceXMode && 
                    (Math.abs(pitchAngleDegrees) <= 
                    Math.abs(kOonBalanceAngleThresholdDegrees))) {
                autoBalanceXMode = false;
            }
            if ( !autoBalanceYMode && 
                (Math.abs(pitchAngleDegrees) >= 
                Math.abs(kOffBalanceAngleThresholdDegrees))) {
                autoBalanceYMode = true;
            }
            else if ( autoBalanceYMode && 
                    (Math.abs(pitchAngleDegrees) <= 
                    Math.abs(kOonBalanceAngleThresholdDegrees))) {
                autoBalanceYMode = false;
            }
            
            // Control drive system automatically, 
            // driving in reverse direction of pitch/roll angle,
            // with a magnitude based upon the angle
            

            if ( autoBalanceXMode ) {
                double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
                xAxisRate = Math.sin(pitchAngleRadians) * -1;
            }
            if ( autoBalanceYMode ) {
                double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
                yAxisRate = Math.sin(rollAngleRadians) * -1;
            }
            myRobot.mecanumDrive_Cartesian(xAxisRate, yAxisRate, stick.getTwist(),0);
            Timer.delay(0.005);		// wait for a motor update time
        }
}
}
*/