// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ClawModeToggleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighAuton extends SequentialCommandGroup {
  /** Creates a new ScoreHighAuton. */
  public ScoreHighAuton(String gamePiece, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    
    if (gamePiece.equalsIgnoreCase(Constants.SCORE_CONE)) {
      addCommands(
        new ArmAuton(armSubsystem, Constants.ARM_HIGH_STATE),
        new ClawModeToggleCommand(clawSubsystem, Constants.CLAW_OPEN_STATE),
        new WaitCommand(0.2)
      );
    } else if (gamePiece.equalsIgnoreCase(Constants.SCORE_CUBE)) {
      addCommands(
        new ArmAuton(armSubsystem, Constants.ARM_MID_STATE),
        new ParallelDeadlineGroup(
          new WaitCommand(0.25),
          new ClawWheelAuton(clawSubsystem, Constants.OUT)
          )
      );
    } else {
      //do nothing
    }

    
  }
}
