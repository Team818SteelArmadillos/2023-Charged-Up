package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PistonClawSubsystem;

public class ClawCommand extends CommandBase {

  PistonClawSubsystem pistonClawSubsystem;
  LEDSubsystem ledSubsystem;

  public ClawCommand(PistonClawSubsystem sub1, LEDSubsystem sub2) {
    addRequirements(sub1, sub2);
    pistonClawSubsystem = sub1;
    ledSubsystem = sub2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pistonClawSubsystem.toggle();
    ledSubsystem.toggle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
     
}