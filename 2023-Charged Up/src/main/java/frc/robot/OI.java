package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI {

    private static CommandXboxController driveController;
    private static CommandXboxController operatorController;

    //public OI () 
    static {
      operatorController = new CommandXboxController(Constants.operatorControllerPort);
      driveController = new CommandXboxController(Constants.DRIVER_PORT);
    }

    public static CommandXboxController getDriver() {
      return driveController;
    }

    public static CommandXboxController getOperator() {
      return operatorController;
    }
}