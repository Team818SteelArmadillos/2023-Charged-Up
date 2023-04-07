// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private double speed;
  private boolean getLeftBumper;
  private boolean getRightBumper;

  private int double_press_counter;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem sub) {
    addRequirements(sub);
    intakeSubsystem = sub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double_press_counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    getLeftBumper = OI.getDriver().getLeftBumper();
    getRightBumper = OI.getDriver().getRightBumper();
    speed = 1.0;

    //set intake speed
    if (getRightBumper){
      if (intakeSubsystem.isIntakeUnlocked()) {
        intakeSubsystem.intakeExtend();
      }
      intakeSubsystem.setIntakeSpeed(-speed);
    } else if (getLeftBumper) {
      if (intakeSubsystem.isIntakeUnlocked()) {
        intakeSubsystem.intakeExtend();
      }
      intakeSubsystem.setIntakeSpeed(speed);
    } else {
      intakeSubsystem.setIntakeSpeed(0);
      intakeSubsystem.intakeRetract();
    }

    // Check double press for locking
    if(getLeftBumper || getRightBumper) {
      if (double_press_counter <= 20) {
        intakeSubsystem.intakeUnlock();
      }
      double_press_counter = 0;
    } else {
      double_press_counter++;
      if (double_press_counter >= 40) {
        intakeSubsystem.intakeLock();
        double_press_counter = 40;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
