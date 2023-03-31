// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.LimeNetwork;

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
  Rotation2d m_targetAngle;
  
  CTRSwerveSubsystem m_drivetrain;
  LimeNetwork m_limelight;
  Timer timeout;

  PIDController m_xPid;
  PIDController m_yPid;

  public DriveToPositionAuton(double targetX, double targetY, Rotation2d targetAngle, CTRSwerveSubsystem drivetrain, LimeNetwork limelight, boolean useLime) {
    addRequirements(drivetrain);

    timeout = new Timer();

    m_xPid = new PIDController(1.7, 0, 0.0);
    m_yPid = new PIDController(1.7, 0, 0.0);

    m_xPid.setTolerance(0.01);
    m_yPid.setTolerance(0.01);
    
    m_targetAngle = targetAngle;
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    
    if (useLime) {
      m_targetX = targetX + m_limelight.getXOffset();
      m_targetY = targetY + m_limelight.getYOffset();  
    } else {
      m_targetX = targetX;
      m_targetY = targetY;
    }
    
    // SmartDashboard.putNumber("xTarget", 0.0);
    // SmartDashboard.putNumber("yTarget", 0.0);

    // SmartDashboard.putNumber("p", 0.0);
    // SmartDashboard.putNumber("i", 0.0);
    // SmartDashboard.putNumber("d", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeout.reset();
    timeout.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX = m_drivetrain.getCTRSwerveDrivetrain().getPoseMeters().getX();  //Just get these values from the odometry
    double currentY = m_drivetrain.getCTRSwerveDrivetrain().getPoseMeters().getY();
    // m_targetX = SmartDashboard.getNumber("xTarget", 0.0);
    // m_targetY = SmartDashboard.getNumber("yTarget", 0.0);
    double xSpeed = 0;
    double ySpeed = 0;
    // m_xPid.setPID(SmartDashboard.getNumber("p", 0.0), SmartDashboard.getNumber("i", 0.0), SmartDashboard.getNumber("d", 0.0));
    // m_yPid.setPID(SmartDashboard.getNumber("p", 0.0), SmartDashboard.getNumber("i", 0.0), SmartDashboard.getNumber("d", 0.0));
    
    xSpeed = m_xPid.calculate(currentX, m_targetX);
    ySpeed = m_yPid.calculate(currentY, m_targetY);

    //SmartDashboard.putNumber("ySpeedOut", ySpeed);

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
    return (m_xPid.atSetpoint() && m_yPid.atSetpoint()) || timeout.hasElapsed(2.5);
  }
}
