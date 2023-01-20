// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.oi.*;

public class OI {

    Joystick leftJoyStick, rightJoyStick;
    XboxController  gamePad, gamePadDriver;

    JoystickButton intakeButton;
    JoystickButton elevatorButton;

    public OI() {

    leftJoyStick = new Joystick(leftJoystickPort);
    rightJoyStick = new Joystick(rightJoystickPort);
    gamePad = new XboxController(gamePadPort);
    gamePadDriver = new XboxController(gamePadDriverPort);
    elevatorButton = new JoystickButton(gamePad, 7);

}

      public boolean getDPadUp() {
        return (gamePad.getPOV() == 0);
      }

      public boolean getDPadDown() {
        return (gamePad.getPOV() == 180);
      }

      public boolean getDPadRight(){
        return (gamePad.getPOV() == 270);
      }

      public boolean getDpadLeft(){
        return (gamePad.getPOV() == 90);
      }

      public boolean getXButton() {
        return gamePad.getXButton();
      }    

      public boolean getYButton() {
        return gamePad.getYButton();
      }

      public boolean getAButton() {
        return gamePad.getAButton();
      }

      public boolean getBButton() {
        return gamePad.getBButton();
      }

      public boolean getLeftBumper() {
        return gamePad.getLeftBumperPressed();
      }

      public boolean getRightBumper() {
        return gamePad.getRightBumper();
      }

      public boolean getRightTrigger() {
        return gamePad.getRightTriggerAxis() > 0.5;
      }

      public boolean getLeftTrigger() {
        return gamePad.getLeftTriggerAxis() > 0.5;
      }

      public boolean getBackButton() {
        return gamePad.getBackButton();
      }
      
      public boolean getStartButton(){
        return gamePad.getStartButton();
      }

      public double getleftYAxis() {
        if (Math.abs(leftJoyStick.getY()) > 0.05){
          return leftJoyStick.getY();
        } else {
          return 0;
        }
      }
    
      public double getrightYAxis() {
        if (Math.abs(rightJoyStick.getY()) > 0.05){
          return rightJoyStick.getY();
        } else {
          return 0;
        }
      }
    
      public double getleftXAxis() {
        if (Math.abs(leftJoyStick.getX()) > 0.05){
          return leftJoyStick.getX();
        } else {
          return 0;
        }
      }
      
      public double getrightXAxis() {
        if (Math.abs(rightJoyStick.getX()) > 0.05){
          return rightJoyStick.getX();
        } else {
          return 0;
        }
      }

      public double getgamepadleftXAxis() {
        if (Math.abs(gamePad.getLeftX()) > 0.05){
          return gamePad.getLeftX();
        } else {
          return 0;
        }
      }
      public double getgamepadrightXAxis() {
        if (Math.abs(gamePad.getRightX()) > 0.05){
          return  gamePad.getRightX();
        } else {
          return 0;
        }
      }

      public boolean getRightJoystick2(){
      return (rightJoyStick.getRawButton(2));
      }

      public boolean getRightJoystick8(){
        return (rightJoyStick.getRawButton(8));
      }

      public boolean getRightJoystick7(){
        return (rightJoyStick.getRawButton(7));
      }

      public boolean getRightJoystick5(){
        return (rightJoyStick.getRawButton(5));
      }

      public boolean getRightJoystick11(){
        return (rightJoyStick.getRawButton(11));
      }

    }