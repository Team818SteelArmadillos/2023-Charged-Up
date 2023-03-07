// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.SwerveDrivetrain;

import java.io.*;



public class ManualPathPlanningCommand extends CommandBase {
  static int AutonNumber;
  public static boolean Commandfinished = false;
  private SwerveDrivetrain m_swerveDrivetrain;
  private AutoDrive m_autoDrive;
  
  //First slot is for declaring either coordinate, rotation, or other types of movement. Other is for coordinates or additonal data depending on movement type.
  static double[][] coordinates = new double[5][3];
  public ManualPathPlanningCommand() {
    addRequirements(m_swerveDrivetrain, m_autoDrive);
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
  public void autonRun(){
    for(var i = 0; i < coordinates.length; i++){
      while(!Commandfinished){
        switch((int)coordinates[i][0]){
          //issues with static references, autonrun might need to be an object???
          //still not sure how to actually call drive method, is the swerve drive object created before auton?
          case 0:
            m_swerveDrivetrain.drive(AutoDrive.autoDrive(coordinates[i][1], coordinates[i][2], m_swerveDrivetrain.getPose().getX(), m_swerveDrivetrain.getPose().getY())); 
          case 1:
            m_autoDrive.autorotate(coordinates[i][1]); //Assumed rotation if coordinates[i][0] = 1, coordinates[i][1] should contain desired direction.
        }
      }
    }
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
