// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CTRSwerveSubsystem;

public class DriveToPositionAuton extends CommandBase {
  /** Creates a new DriveToPosition. */
  
  double m_targetX;
  double m_targetY;
  Rotation2d m_targetAngle;
  
  CTRSwerveSubsystem m_drivetrain;

  PIDController m_xPid;
  PIDController m_yPid;

  public DriveToPositionAuton(double targetX, double targetY, Rotation2d targetAngle, CTRSwerveSubsystem drivetrain) {
    addRequirements(drivetrain);

    m_xPid = new PIDController(1.5, 0, 0.2);
    m_yPid = new PIDController(1.5, 0, 0.2);

    m_xPid.setTolerance(0.1);
    m_yPid.setTolerance(0.1);
    
    m_targetX = targetX;
    m_targetY = targetY;
    m_targetAngle = targetAngle;
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    
    xSpeed = m_xPid.calculate(currentX, m_targetX) * Constants.MAX_SPEED;
    ySpeed = m_yPid.calculate(currentY, m_targetY) * Constants.MAX_SPEED;

    //SmartDashboard.putNumber("ySpeedOut", ySpeed);

    m_drivetrain.getCTRSwerveDrivetrain().driveFullyFieldCentric(xSpeed, ySpeed, m_targetAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.getCTRSwerveDrivetrain().driveStopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_xPid.atSetpoint() && m_yPid.atSetpoint());
  }
}
