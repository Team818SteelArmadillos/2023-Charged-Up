// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.subsystems.CTRSwerveSubsystem;

public class DriveToPlatformAuton extends CommandBase {

  private CTRSwerveDrivetrain m_drivetrain;
  private Rotation2d m_lastCurrentAngle;
  private int m_direction;
  private int inlcine_counter;
  private Timer m_timeoutTimer;

  /** Creates a new BalanceAuton. */
  public DriveToPlatformAuton(int direction, Rotation2d rotation, CTRSwerveSubsystem drivetrain) {
    addRequirements(drivetrain);

    m_drivetrain = drivetrain.getCTRSwerveDrivetrain();
    m_direction = direction;
    m_timeoutTimer = new Timer();
    inlcine_counter = 0;
    
    m_lastCurrentAngle = rotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timeoutTimer.reset();
    m_timeoutTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // drive at 40% speed
      m_drivetrain.driveFullyFieldCentric((m_direction * 0.6) * Constants.MAX_SPEED, 0.0, m_lastCurrentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timeoutTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_drivetrain.getPitch()) >= Constants.MINIMUM_CHARGE_STATION_ANGLE_THRESH) {
      inlcine_counter++;
    } else {
      inlcine_counter = 0;
    }

    return  inlcine_counter >= 20 || m_timeoutTimer.hasElapsed(3);
  }
}
