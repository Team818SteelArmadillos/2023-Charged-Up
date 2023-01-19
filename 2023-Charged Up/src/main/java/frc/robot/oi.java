package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {

    private static XboxController driveController;
    private static XboxController operatorController;

    //public OI () 
    static {
      driveController = new XboxController(Constants.driverControllerPort);
      operatorController = new XboxController(Constants.operatorControllerPort);
    }

    public static XboxController getDriver() {
      return driveController;
    }

    public static XboxController getOperator() {
      return operatorController;
    }
}