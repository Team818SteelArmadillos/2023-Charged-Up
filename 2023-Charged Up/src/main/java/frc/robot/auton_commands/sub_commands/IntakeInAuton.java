// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


public class IntakeInAuton extends CommandBase {
  private IntakeSubsystem m_IntakeSubsystem;
  private Timer timer;
  private PowerDistribution intakepower;
  private double normalPower;

  /** Creates a new IntakeAuton. */
  public IntakeInAuton(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    m_IntakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    //need to figure out pdp port and type
    intakepower = new PowerDistribution();
    normalPower = intakepower.getCurrent(8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.intakeExtend();
    m_IntakeSubsystem.setIntakeSpeed(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.intakeRetract();
    m_IntakeSubsystem.setIntakeSpeed(0.15);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.hasElapsed(5)||intakepower.getCurrent(8) > (normalPower + 0.25) ){
      return true;
    }
    return false;
  }
}
