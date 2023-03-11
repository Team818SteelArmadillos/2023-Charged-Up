package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;


//im desperately trying to figure out how to PID lol - casey
//if you can add anything or help at all PLEASE DO
//I'm gonna do some stuff... - Aiden

//goober


public class ChargingStationPIDSubsystem extends SubsystemBase {

    public Pigeon2 pigeon;
    public double currentPitch;
    public double goalPitch = 0;
    public PIDController pid;
    public double dm; //placeholder for driving motors

    //initialize here! 😂
    public ChargingStationPIDSubsystem() {
        SmartDashboard.putNumber("Pitch", pigeon.getPitch());
        currentPitch = pigeon.getPitch();
        pid = new PIDController(Constants.csP, Constants.csI, Constants.csD);
        
}

public void fixPitch(){
    dm.set(ControlMode.PercentOutput, pid.calculate(currentPitch, goalPitch));
}
        
}
