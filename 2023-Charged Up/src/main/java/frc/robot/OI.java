package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

    private static XboxController driveController;
    private static PS4Controller operatorController;

    //public OI () 
    static {
      operatorController = new PS4Controller(Constants.operatorControllerPort);
    }

    public static XboxController getDriver() {
      return driveController;
    }

    public static PS4Controller getOperator() {
      return operatorController;
    }
}