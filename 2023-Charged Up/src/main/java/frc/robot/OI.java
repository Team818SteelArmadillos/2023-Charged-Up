package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class OI {

    private static XboxController driveController;
    private static CommandPS4Controller operatorController;

    //public OI () 
    static {
      operatorController = new CommandPS4Controller(Constants.operatorControllerPort);
    }

    public static XboxController getDriver() {
      return driveController;
    }

    public static CommandPS4Controller getOperator() {
      return operatorController;
    }
}