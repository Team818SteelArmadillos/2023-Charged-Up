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

public class SlowDriveCommand extends CommandBase {

  private CTRSwerveDrivetrain m_drivetrain;
  private Rotation2d m_lastCurrentAngle;
  private int m_direction;
  private double m_time;
  private Timer m_timeoutTimer;
  
  /** Creates a new SlowDriveCommand. */
  public SlowDriveCommand(int direction, double time, CTRSwerveSubsystem drivetrain) {
    m_drivetrain = drivetrain.getCTRSwerveDrivetrain();
    m_direction = direction;
    m_timeoutTimer = new Timer();
    m_time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lastCurrentAngle = m_drivetrain.getPoseMeters().getRotation();
    m_timeoutTimer.reset();
    m_timeoutTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // drive at 40% speed
      m_drivetrain.driveFullyFieldCentric((m_direction * 0.2) * Constants.MAX_SPEED, 0.0, m_lastCurrentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timeoutTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_timeoutTimer.hasElapsed(m_time);
  }
}

