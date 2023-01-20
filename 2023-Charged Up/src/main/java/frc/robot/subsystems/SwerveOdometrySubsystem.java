package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import java.lang.reflect.Method;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants.*;


public class SwerveOdometrySubsystem extends SubsystemBase {
  public static Timer odometryTimer = new Timer();
  private static double timer;
  static double[] coordinates = {0,0};

  public SwerveOdometrySubsystem(){
  }
  
   
  public static double[] getCoordinates() {
    return coordinates;
  }
    private static double[] deltaRobot(){
      double speed1 = timer * ((FRONTRIGHTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed2 = timer * ((FRONTLEFTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed3 = timer * ((BACKRIGHTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double speed4 = timer * ((BACKLEFTDRIVEMOTOR.getSelectedSensorVelocity() * DISTANCEPERPULSE * VELOCITYCALCULATIONPERSECOND)/GEARRATIO);
      double angle1 = falconToDegrees(FRONTRIGHTTURNMOTOR.getSelectedSensorVelocity()) + Math.toDegrees(PIGEON.getAngle());
      double angle2 = falconToDegrees(FRONTLEFTTURNMOTOR.getSelectedSensorVelocity()) + Math.toDegrees(PIGEON.getAngle());
      double angle3 = falconToDegrees(BACKRIGHTTURNMOTOR.getSelectedSensorVelocity()) + Math.toDegrees(PIGEON.getAngle());
      double angle4 = falconToDegrees(BACKLEFTTURNMOTOR.getSelectedSensorVelocity()) + Math.toDegrees(PIGEON.getAngle());

      double deltaX = ((speed1 *Math.cos(Math.toRadians(angle1))) + (speed2 * Math.cos(Math.toRadians(angle2))) + (speed3 *
      Math.cos(Math.toRadians(angle3))) + (speed4 * Math.cos(Math.toRadians(angle4))))/4;

      double deltaY = ((speed1 *Math.sin(Math.toRadians(angle1))) + (speed2 * Math.sin(Math.toRadians(angle2))) + (speed3 *
      Math.sin(Math.toRadians(angle3))) + (speed4 * Math.sin(Math.toRadians(angle4))))/4;

      double delta[] = {deltaX, deltaY};
      return delta;
    }
  
  @Override
  public void periodic() {
    timer = odometryTimer.get();
    odometryTimer.stop();
    odometryTimer.reset();
    odometryTimer.start();
    double[] delta = deltaRobot();
    coordinates[0]+=delta[0];
    coordinates[1]+=delta[1];
  }
}
