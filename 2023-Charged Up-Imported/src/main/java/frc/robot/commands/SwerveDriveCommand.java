// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class SwerveDriveCommand extends CommandBase {
  private double m_rotation;
  private Translation2d m_translation;
  private boolean m_fieldRelative;
  private boolean m_openLoop;
   
  private SwerveDriveSubsystem m_swerveDriveSubsystem;
  private XboxController m_driverController;
  private double m_driveAxis;
  private double m_strafeAxis;
  private double m_rotationAxis;

  private SlewRateLimiter m_xAxisARateLimiter;
  private SlewRateLimiter m_yAxisARateLimiter;
  //private double 
  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand() {
    addRequirements(Robot.m_robotcontainer.m_swerveDriveSubsystem);

    m_driverController = Robot.m_oi.gamePadDriver;
    m_driveAxis = Robot.m_oi.gamePadDriver.getLeftY();
    m_strafeAxis = Robot.m_oi.gamePadDriver.getLeftX();
    m_rotationAxis = Robot.m_oi.gamePadDriver.getRightX();
    m_fieldRelative = true;
    m_openLoop = true;

    m_xAxisARateLimiter = new SlewRateLimiter(20); //replace the 2 with the a rate limiter from constants
    m_yAxisARateLimiter = new SlewRateLimiter(20);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yAxis = -m_driverController.getLeftY();
    double xAxis = -m_driverController.getLeftX();
    double rAxis = -Robot.m_oi.gamePadDriver.getLeftX();
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < 10) ? 0 : yAxis; //replace the 10 with stick deadband
        xAxis = (Math.abs(xAxis) < 10) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < 10) ? 0 : rAxis;

        /* Square joystick inputs */
        double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
        double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
        double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;

        /* Filter joystick inputs using slew rate limiter */
        double yAxisFiltered = m_yAxisARateLimiter.calculate(yAxisSquared);
        double xAxisFiltered = m_xAxisARateLimiter.calculate(xAxisSquared);

        /* Input variables into drive methods */
        m_translation = new Translation2d(yAxisFiltered, xAxisFiltered);//replace the 5 with the max speed constant
        m_rotation = rAxisSquared * Constants.MAX_ANGULAR_VELOCITY; //replace the 5 with the max angular velocitt constant value
        m_swerveDriveSubsystem.drive(m_translation, m_rotation, m_fieldRelative, m_openLoop);
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