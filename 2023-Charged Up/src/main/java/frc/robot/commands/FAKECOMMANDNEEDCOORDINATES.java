package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.SwerveDrivetrain;



public class ManualPathPlanningCommand extends CommandBase {
  static int AutonNumber;
  private static SwerveDrivetrain m_swerveDrivetrain;
  private static AutoDrive m_autoDrive;
  private static int commandIndex = 0;
  static double[][] coordinates;
  
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
    switch(Robot.m_chooser.getSelected()){
      case 0:{
        double[][] autonCoords = {{0, 70.78, -255.11}, {0, 70.78, -172.61}};
        coordinates = autonCoords;
        break;
      }
      case 1:{
        //Should be blue left when finished.
        double[][] autonCoords = {{0, 137.61,-255.11}, {}, {0, 121.61, -255.11}, {0, 121.61, -64.62}, {}, {0, 121.61, -255.11}, {0, 93.61, -255.11}};
        coordinates = autonCoords;
        break;
      }
      default:{
        double[][] autonCoords = {{5, 0, 0}};
        coordinates = autonCoords;
        break;
      }
    }
  }
  
  public static void AutonRun(){
    if(commandIndex == 0){
      Rotation2d startRotation = new Rotation2d();
      Pose2d startingCoords = new Pose2d(coordinates[0][1], coordinates[0][2], startRotation);
      m_swerveDrivetrain.resetOdometry(startingCoords);
      commandIndex=+1;
    }
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

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    //might need to be >
    return commandIndex == coordinates.length || coordinates[commandIndex][0] == 5;
  }
}