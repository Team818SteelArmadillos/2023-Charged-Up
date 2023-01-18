package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;

public class OI {

    private static XboxController driveController;
    private static XboxController cmdController;

    //public OI () 
    static {
      driveController = new XboxController(1);
      cmdController = new XboxController(2);
    }

    public static XboxController getDriver() {
      return driveController;
    }

    public static XboxController getCMDController() {
      return cmdController;
    }

}