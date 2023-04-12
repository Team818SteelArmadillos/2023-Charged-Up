// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CTRSwerveSubsystem;

public class DriveToPositionAuton extends CommandBase {
  /** Creates a new DriveToPosition. */


  /*               positive x
   *                   ^
   *                   |
   * positive y  <-----+----->
   *                   |
   *                   v
   * 
   * 
   */

  
  double m_targetX;
  double m_targetY;
  double timeoutTime;
  Rotation2d m_targetAngle;
  
  CTRSwerveSubsystem m_drivetrain;
  int point_to_direction;
  Timer timeout;

  PIDController m_xPid;
  PIDController m_yPid;

  public DriveToPositionAuton(double targetX, double targetY, Rotation2d targetAngle, CTRSwerveSubsystem drivetrain) {
    addRequirements(drivetrain);

    timeout = new Timer();

    m_xPid = new PIDController(1.7, 0, 0.0);
    m_yPid = new PIDController(1.7, 0, 0.0);

    m_xPid.setTolerance(0.05);
    m_yPid.setTolerance(0.05);
    
    m_targetX = targetX;
    m_targetY = targetY;
    m_targetAngle = targetAngle;
    m_drivetrain = drivetrain;

    point_to_direction = 0;
  }

  public DriveToPositionAuton(int intakeDirection, double targetX, double targetY, CTRSwerveSubsystem drivetrain) {
    addRequirements(drivetrain);

    timeout = new Timer();

    m_xPid = new PIDController(1.8, 0, 0.0);
    m_yPid = new PIDController(1.8, 0, 0.0);

    m_xPid.setTolerance(0.01);
    m_yPid.setTolerance(0.01);
    
    m_targetX = targetX;
    m_targetY = targetY;
    m_drivetrain = drivetrain;

    point_to_direction = intakeDirection;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //calculate timeout based on distance
    timeoutTime = Math.sqrt(Math.pow(m_targetX - m_drivetrain.getCTRSwerveDrivetrain().getPoseMeters().getX(), 2) 
    + Math.pow(m_targetY - m_drivetrain.getCTRSwerveDrivetrain().getPoseMeters().getY(), 2)) / 1.9;

    timeout.reset();
    timeout.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX = m_drivetrain.getCTRSwerveDrivetrain().getPoseMeters().getX();  //Just get these values from the odometry
    double currentY = m_drivetrain.getCTRSwerveDrivetrain().getPoseMeters().getY();

    double xSpeed = 0;
    double ySpeed = 0;
    
    xSpeed = m_xPid.calculate(currentX, m_targetX);
    ySpeed = m_yPid.calculate(currentY, m_targetY);

    switch (point_to_direction) {
      case 0: // do nothing
        break;
      case 1: 
        m_targetAngle = new Rotation2d(-(m_targetX - currentX), -(m_targetY - currentY));
        break;
      case 2: 
        m_targetAngle = new Rotation2d(-(m_targetX - currentX), -(m_targetY - currentY)).minus(new Rotation2d(0.0, 1.0));
        break;
    }

    m_drivetrain.getCTRSwerveDrivetrain().driveFullyFieldCentric(xSpeed, ySpeed, m_targetAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.getCTRSwerveDrivetrain().driveStopMotion();
    timeout.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_xPid.atSetpoint() && m_yPid.atSetpoint()) || timeout.hasElapsed(timeoutTime);
  }
}
