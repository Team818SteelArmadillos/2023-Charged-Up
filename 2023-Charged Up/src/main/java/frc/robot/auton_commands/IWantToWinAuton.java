// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ClawModeToggleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IWantToWinAuton extends SequentialCommandGroup {
  /** Creates a new IWantToWinAuton. */
  public IWantToWinAuton(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmAuton(armSubsystem, Constants.ARM_HIGH_STATE),
      new WaitCommand(0.5),
      new ClawModeToggleCommand(clawSubsystem),
      new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE)

    );
  }
}
