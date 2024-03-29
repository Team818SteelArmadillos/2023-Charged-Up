// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClawWheelsSubsystem;

public class ClawWheelAuton extends CommandBase {
  /** Creates a new ClawWheelAuton. */
  private ClawWheelsSubsystem m_ClawWheelsSubsystem;
  private Timer endTimer;
  private int endAfter;
  private boolean in;
  public ClawWheelAuton(int time, ClawWheelsSubsystem clawWheel, boolean in) {
    m_ClawWheelsSubsystem = clawWheel;
    endTimer = new Timer();
    endAfter = time;
    this.in = in;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer.reset();
    endTimer.start();
    if(in){
      m_ClawWheelsSubsystem.setIntakeSpeed(Constants.clawWheelForawrdSpeed);
    }else{
      m_ClawWheelsSubsystem.setIntakeSpeed(Constants.clawWheelReverseSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endTimer.stop();
    m_ClawWheelsSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endTimer.hasElapsed(endAfter);
  }
}
