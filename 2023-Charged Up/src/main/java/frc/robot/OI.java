package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI {

    private static XboxController driveController;
    private static XboxController xopController;
    private static CommandXboxController operatorController;
    private static CommandXboxController testingController;

    //public OI () 
    static {
      operatorController = new CommandXboxController(Constants.operatorControllerPort);
      driveController = new XboxController(Constants.DRIVER_PORT);
      xopController = new XboxController(Constants.operatorControllerPort);
      testingController = new CommandXboxController(3);
    }

    public static CommandXboxController getTesting(){
      return testingController;
    }
    public static XboxController getDriver() {
      return driveController;
    }

    public static CommandXboxController getOperator() {
      return operatorController;
    }

    public static XboxController getXOperator() {
      return xopController;
    }
}