package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;

public class OI {

    private static XboxController driveController;
    private static XboxController operatorController;

    //public OI () 
    static {
      driveController = new XboxController(DRIVER_CONTROLLER_PORT);
      operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);
    }

    public static XboxController getDriver() {
      return driveController;
    }

    public static XboxController getOperator() {
      return operatorController;
    }

}