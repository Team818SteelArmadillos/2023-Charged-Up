package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI {

    private static XboxController driveController;
    private static CommandXboxController operatorController;

    //public OI () 
    static {
      operatorController = new CommandXboxController(Constants.operatorControllerPort);
    }

    public static XboxController getDriver() {
      return driveController;
    }

    public static CommandXboxController getOperator() {
      return operatorController;
    }
}