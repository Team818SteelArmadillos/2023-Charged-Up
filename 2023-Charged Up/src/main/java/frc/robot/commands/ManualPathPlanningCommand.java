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
  
  public ManualPathPlanningCommand() {
    addRequirements(m_swerveDrivetrain, m_autoDrive);
  }

  /**Coordinates are in sets of 3:
   * Number 1 ditates command type:
   *  0 for coordinates
   *  1 for rotation
   *  5 for last command
   * Remaining slots clarify command:
   *  Insert coordinates for coordinates
   *  Insert desired direction for rotation
   * Leave unnecessary slots as 0s,
  */
  public static void chooseAuton(){
    String autonCoords;
    switch(Robot.m_chooser.getSelected()){
      case 0:
      autonCoords = "0450560320810925";
      case 1:
      autonCoords = "0550580320820925";
      default:
      autonCoords = "5000000000000000";
    }
    int y = 0;
    for(var i = 0; i < 5; i++){
      for(var x = 0; x < 3; x++){
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
        case 0:
        //open loop might be more accurate??
          m_swerveDrivetrain.drive(AutoDrive.autoDrive(coordinates[commandIndex][1], coordinates[commandIndex][2], m_swerveDrivetrain.getPose().getX(), m_swerveDrivetrain.getPose().getY()), 0, true, true); 
        case 1:
          m_swerveDrivetrain.drive(null, m_autoDrive.autorotate(coordinates[commandIndex][1]), true, true); //Assumed rotation if coordinates[i][0] = 1, coordinates[i][1] should contain desired direction.
        
      }
    }
  }

  public static boolean checkStatus(double[][] coordinates, int index){
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
