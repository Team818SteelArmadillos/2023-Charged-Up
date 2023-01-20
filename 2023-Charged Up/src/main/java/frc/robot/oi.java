package frc.robot;

//Imports
import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.oi.*;

public class oi {
  //defines GamePad
  XboxController gamePadOperator, gamePadDriver;

  public oi() {
    
    //Defines Driver/Operator Port
    gamePadDriver = new XboxController(gamePadDriverPort);
    gamePadOperator = new XboxController(gamePadOperatorPort);

  }

  //Driver Buttons
  public boolean getDpadUpDriver(){
    return (gamePadDriver.getPOV() == 0);
  }
  public boolean getDpadDownDriver(){
    return (gamePadDriver.getPOV() == 180);
  }
  public boolean getDpadLeftDriver(){
    return (gamePadDriver.getPOV() == 270);
  }
  public boolean getDpadRightDriver(){
    return (gamePadDriver.getPOV() == 90);
  }
  public boolean getXButtonDriver(){
    return gamePadDriver.getXButton();
  }
  public boolean getYButtonDriver(){
    return gamePadDriver.getYButton();
  }
  public boolean getAButtonDriver(){
    return gamePadDriver.getAButton();
  }
  public boolean getBButtonDriver(){
    return gamePadDriver.getBButton();
  }
  public boolean getStartButtonDriver(){
    return gamePadDriver.getStartButton();
  }
  public boolean getBackButtonDriver(){
    return gamePadDriver.getBackButton();
  }
  public boolean getLeftBumperDriver(){
    return gamePadDriver.getRawButtonPressed(5);
  }
  public boolean getRightBumperDriver(){
    return gamePadDriver.getRawButtonPressed(6);
  }
  public boolean getRightTriggerDriver(){
    return gamePadDriver.getRawButtonPressed(3);
  }
  public boolean getLeftTriggerDriver(){
    return gamePadDriver.getRawButtonPressed(2);
  }
  
  //Driver Joysticks
  public double getLeftXDriver(){
    return gamePadDriver.getLeftX();
  }
  public double getLeftYDriver(){
    return gamePadDriver.getLeftY();
  }
  public double getRightXDriver(){
    return gamePadDriver.getRightX();
  }
  public double getRightYDriver(){
    return gamePadDriver.getRightY();
  }
  public boolean getLeftStickButtonDriver(){
    return gamePadDriver.getLeftStickButton();
  }
  public boolean getRightStickButtonDriver(){
    return gamePadDriver.getRightStickButton();
  }
  
  //Operator Buttons
  public boolean getDpadUpOperator(){
    return (gamePadOperator.getPOV() == 0);
  }
  public boolean getDpadDownOperator(){
    return (gamePadOperator.getPOV() == 180);
  }
  public boolean getDpadLeftOperator(){
    return (gamePadOperator.getPOV() == 270);
  }
  public boolean getDpadRightOperator(){
    return (gamePadOperator.getPOV() == 90);
  }
  public boolean getXButtonOperator(){
    return gamePadOperator.getXButton();
  }
  public boolean getYButtonOperator(){
    return gamePadOperator.getYButton();
  }
  public boolean getAButtonOperator(){
    return gamePadOperator.getAButton();
  }
  public boolean getBButtonOperator(){
    return gamePadOperator.getBButton();
  }
  public boolean getStartButtonOperator(){
    return gamePadOperator.getStartButton();
  }
  public boolean getBackButtonOperator(){
    return gamePadOperator.getBackButton();
  }
  public boolean getLeftBumperOperator(){
    return gamePadOperator.getRawButtonPressed(5);
  }
  public boolean getRightBumperOperator(){
    return gamePadOperator.getRawButtonPressed(6);
  }
  public boolean getRightTriggerOperator(){
    return gamePadOperator.getRawButtonPressed(3);
  }
  public boolean getLeftTriggerOperator(){
    return gamePadOperator.getRawButtonPressed(2);
  }
  
  //Operator Joysticks
  public double getLeftXOperator(){
    return gamePadOperator.getLeftX();
  }
  public double getLeftYOperator(){
    return gamePadOperator.getLeftY();
  }
  public double getRightXOperator(){
    return gamePadOperator.getRightX();
  }
  public double getRightYOperator(){
    return gamePadOperator.getRightY();
  }
  public boolean getLeftStickButtonOperator(){
    return gamePadOperator.getLeftStickButton();
  }
  public boolean getRightStickButtonOperator(){
    return gamePadOperator.getRightStickButton();
  }
}
