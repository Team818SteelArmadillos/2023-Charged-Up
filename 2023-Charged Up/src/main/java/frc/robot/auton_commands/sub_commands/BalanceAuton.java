// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.subsystems.CTRSwerveSubsystem;

public class BalanceAuton extends CommandBase {

  private CTRSwerveDrivetrain m_drivetrain;
  private Rotation2d m_lastCurrentAngle;
  
  private int balance_counter;
  private int charge_station_stationary_counter;
  private double m_lastPitch;

  private PIDController balancePID;

  /** Creates a new BalanceAuton. */
  public BalanceAuton(CTRSwerveSubsystem drivetrain) {
    addRequirements(drivetrain);

    m_drivetrain = drivetrain.getCTRSwerveDrivetrain();

    balancePID = new PIDController(0.07, 0.0, 0.01);
    balancePID.setTolerance(1.5);

    balance_counter = 0;
    charge_station_stationary_counter = 0;
    m_lastPitch = m_drivetrain.getPitch();

    // SmartDashboard.putNumber("Balance P", 0.0);
    // SmartDashboard.putNumber("Balance I", 0.0);
    // SmartDashboard.putNumber("Balance D", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lastCurrentAngle = m_drivetrain.getPoseMeters().getRotation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //DEBUG
    // balancePID.setP(SmartDashboard.getNumber("Balance P", 0.0));
    // balancePID.setI(SmartDashboard.getNumber("Balance I", 0.0));
    // balancePID.setD(SmartDashboard.getNumber("Balance D", 0.0));

    double pitch = m_drivetrain.getPitch();
    double pitch_delta = m_lastPitch - pitch;
    double output = balancePID.calculate(m_drivetrain.getPitch(), 0);

    //DEBUG
    // SmartDashboard.putNumber("pitch", pitch);
    // SmartDashboard.putNumber("delta_pitch", pitch_delta);

    if (pitch_delta < 0.5) {
      charge_station_stationary_counter++;
    } else {
      charge_station_stationary_counter = 0;
    }

    if (charge_station_stationary_counter >= 20) {
      m_drivetrain.driveFullyFieldCentric(-output * Constants.MAX_SPEED * 0.3, 0.0, m_lastCurrentAngle);
      charge_station_stationary_counter = 20; // prevent int overflow
    } else {
      m_drivetrain.driveStopMotion();
    }

    m_lastPitch = pitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (balancePID.atSetpoint()) {
      balance_counter++;
    } else {
      balance_counter = 0;
    }

    return balance_counter >= 20;
  }
}
