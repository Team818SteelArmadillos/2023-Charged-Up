// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutAuton extends CommandBase {
  private IntakeSubsystem m_IntakeSubsystem;
  private Timer timer;
  private double m_timeout;

  public IntakeOutAuton( IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    m_IntakeSubsystem = intakeSubsystem;
    m_timeout = 2; //Might change depending on time needed to outtake.
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.setIntakeSpeed(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setIntakeSpeed(0);
    m_IntakeSubsystem.intakeRetract();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(m_timeout);
  }
}
