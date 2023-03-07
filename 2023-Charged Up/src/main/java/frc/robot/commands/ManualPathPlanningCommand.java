// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.AutoDrive;
import java.io.*;


public class ManualPathPlanningCommand extends CommandBase {
  static int AutonNumber;
  
  //First slot is for declaring either coordinate, rotation, or other types of movement. Other is for coordinates or additonal data depending on movement type.
  static double[][] coordinates = new double[5][3];
  public ManualPathPlanningCommand() {
  }
  public static void chooseAuton(){
    AutonNumber = Robot.m_chooser.getSelected();
    String autonCoords = "000000000000000";
    //silly little solution
    switch(AutonNumber){
      case 0:
      autonCoords = "045056032081092";
      case 1:
      autonCoords = "055058032082092";
    }
    //should put the above string into the earlier declared list. Local decleration could fix this, but could create problems with it being local? I also assume theres some way to quickly insert all values at once but I couldn't find documentation.
    int y = 0;
    for(var i = 0; i < 4; i++){
      for(var x = 0; x < 2; x++){
        coordinates[i][x] = autonCoords.charAt(y) - 0;
        y+=1;
      }
    }
  }
  public static void autonRun(){
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
