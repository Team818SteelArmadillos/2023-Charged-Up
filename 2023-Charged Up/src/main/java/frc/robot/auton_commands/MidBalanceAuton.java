// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auton_commands.sub_commands.ArmAuton;
import frc.robot.auton_commands.sub_commands.BalanceAuton;
import frc.robot.auton_commands.sub_commands.DriveToPlatformAuton;
import frc.robot.commands.ClawModeToggleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidBalanceAuton extends SequentialCommandGroup {
  /** Creates a new IWantToWinAuton. */
  public MidBalanceAuton(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, CTRSwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    // DO NOT USE THIS AUTON
      new ArmAuton(armSubsystem, Constants.ARM_HIGH_STATE),
      new WaitCommand(0.5),
      new ClawModeToggleCommand(clawSubsystem),
      new WaitCommand(0.3),
      new ParallelCommandGroup(
        new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
        new DriveToPlatformAuton(Constants.FORWARD_DIRECTION, swerveSubsystem)
      ),
      new BalanceAuton(swerveSubsystem)
    );
  }
}
