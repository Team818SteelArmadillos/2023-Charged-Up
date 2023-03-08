// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.SwerveDrivetrain;



public class ManualPathPlanningCommand extends CommandBase {
  static int AutonNumber;
  private static SwerveDrivetrain m_swerveDrivetrain;
  private static AutoDrive m_autoDrive;
  private static int commandIndex = 0;
  static double[][] coordinates = new double[5][3];

  //First slot is for declaring either coordinate, rotation, or other types of movement. Other is for coordinates or additonal data depending on movement type.
  
  public ManualPathPlanningCommand() {
    addRequirements(m_swerveDrivetrain, m_autoDrive);
  }
  /**Inserts a string of numbers to the coordinates list. Coordinates are in sets of 3; 
  first digit declares the type of data (0 is coordinates, 1 is rotation, 5 ends auton, etc), 
  the second and third digits are to offer necessary data. For example, coordinates require an
  x and y, rotation requires the desired direction of the robot. */
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
  public static void AutonRun(){
    for(var i = 0; i < coordinates.length; i++){
      if(checkStatus(coordinates, commandIndex)){
        commandIndex=+1;
      }
      switch((int)coordinates[commandIndex][0]){
        //issues with static references, autonrun might need to be an object???
        //still not sure how to actually call drive method, is the swerve drive object created before auton?
        
        case 0:
          m_swerveDrivetrain.drive(AutoDrive.autoDrive(coordinates[commandIndex][1], coordinates[commandIndex][2], m_swerveDrivetrain.getPose().getX(), m_swerveDrivetrain.getPose().getY()), 0, true, true); 
        case 1:
          m_swerveDrivetrain.drive(null, m_autoDrive.autorotate(coordinates[commandIndex][1]), true, true); //Assumed rotation if coordinates[i][0] = 1, coordinates[i][1] should contain desired direction.
        
      }
    }
  }

  public static boolean checkStatus(double[][] coordinates, int index){
    //checking if command is complete
    switch((int)coordinates[index][0]){
      case 0:
        return Math.sqrt(Math.pow(Math.abs(m_swerveDrivetrain.getPose().getX() - coordinates[index][1]), 2) + Math.pow(Math.abs(m_swerveDrivetrain.getPose().getY() - coordinates[index][2]), 2)) < .5;
      case 1:
        return SwerveDrivetrain.getYaw().getDegrees() - coordinates[index][1] < 1;
    }
    return false;
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
    
    return commandIndex == coordinates.length;
  }
}
