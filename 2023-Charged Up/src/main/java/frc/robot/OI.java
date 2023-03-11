package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI {

    private static CommandXboxController driveController;
    private static CommandXboxController operatorController;

    //public OI () 
    static {
      operatorController = new CommandXboxController(Constants.operatorControllerPort);
    }

    public static CommandXboxController getDriver() {
      return driveController;
    }

    public static CommandXboxController getOperator() {
      return operatorController;
    }
}