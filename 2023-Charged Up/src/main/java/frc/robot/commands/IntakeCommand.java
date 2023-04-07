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
  private boolean left_press_flag;
  private boolean right_press_flag;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem sub) {
    addRequirements(sub);
    intakeSubsystem = sub;

    double_press_counter = 0;
    left_press_flag = false;
    right_press_flag = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    getLeftBumper = OI.getDriver().getLeftBumper();
    getRightBumper = OI.getDriver().getRightBumper();
    speed = 0.6;

    // if (OI.getOperator().povDown().getAsBoolean()) {
    //   intakeSubsystem.intakeExtend();
    // } else if (OI.getOperator().povUp().getAsBoolean()) {
    //   intakeSubsystem.intakeRetract();
    // } else {
    //   intakeSubsystem.intakeStop();
    // }

    

    // if (OI.getOperator().povRight().getAsBoolean()) {
    //   intakeSubsystem.setIntakeSpeed(speed);
    // } else if (OI.getOperator().povLeft().getAsBoolean()) {
    //   intakeSubsystem.setIntakeSpeed(-speed);
    // } else {
    //   intakeSubsystem.setIntakeSpeed(0.0);
    // }

    //set intake speed
    // if (getRightBumper){ // in
    //   if (intakeSubsystem.isIntakeUnlocked() && right_press_flag) {
    //     intakeSubsystem.intakeExtend();
    //   }
    //   intakeSubsystem.setIntakeSpeed(-speed);
    // } else if (getLeftBumper) { // out
    //   if (intakeSubsystem.isIntakeUnlocked() && left_press_flag) {
    //     intakeSubsystem.intakeExtend();
    //   }
    //   intakeSubsystem.setIntakeSpeed(speed);
    // } else {
    //   if (intakeSubsystem.isIntakeUnlocked()) {
    //     intakeSubsystem.intakeRetract();
    //     right_press_flag = false;
    //     left_press_flag = false;
    //   }
    //   intakeSubsystem.setIntakeSpeed(0);
    // }

    // // Check double press for locking
    // if(getLeftBumper) {
    //   if (double_press_counter <= 10 && double_press_counter > 0 && left_press_flag) {
    //     intakeSubsystem.intakeUnlock();
    //   }
    //   double_press_counter = 0;
    //   left_press_flag = true;
    // } else  if (getRightBumper) {
    //   if (double_press_counter <= 10 && double_press_counter > 0 && right_press_flag) {
    //     intakeSubsystem.intakeUnlock();
    //   }
    //   double_press_counter = 0;
    //   right_press_flag = true;
    // } else {
    //   double_press_counter++;
    //   if (double_press_counter > 10) {
    //     intakeSubsystem.intakeLock();
    //     double_press_counter = 11;
    //     left_press_flag = false;
    //     right_press_flag = false;
    //   }
    // }
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
