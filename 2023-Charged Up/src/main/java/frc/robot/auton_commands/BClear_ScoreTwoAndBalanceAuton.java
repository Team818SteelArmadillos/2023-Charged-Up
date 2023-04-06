// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auton_commands.sub_commands.ArmAuton;
import frc.robot.auton_commands.sub_commands.BalanceAuton;
import frc.robot.auton_commands.sub_commands.ClawWheelAuton;
import frc.robot.auton_commands.sub_commands.DriveToGroundIntakeAuton;
import frc.robot.auton_commands.sub_commands.DriveToPlatformAuton;
import frc.robot.auton_commands.sub_commands.DriveToPositionAuton;
import frc.robot.auton_commands.sub_commands.ScoreHighAuton;
import frc.robot.auton_commands.sub_commands.ScoreMidAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BClear_ScoreTwoAndBalanceAuton extends SequentialCommandGroup {
  /** Creates a new IWantToWinAuton. */
  public BClear_ScoreTwoAndBalanceAuton(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, CTRSwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // score
      new ScoreHighAuton(armSubsystem, clawSubsystem),
      //new ClawModeToggleCommand(clawSubsystem),
      new DriveToGroundIntakeAuton(5.0, -0.35, armSubsystem, swerveSubsystem, clawSubsystem),
      new ParallelCommandGroup(
        new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
        new DriveToPositionAuton(-0.1, -0.57, swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(), swerveSubsystem)
      ),
      new ScoreMidAuton(armSubsystem, clawSubsystem),
      new ParallelDeadlineGroup(
        new WaitCommand(0.2), 
        new ClawWheelAuton(clawSubsystem, false)
      ),
      new ParallelCommandGroup(
        new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
        new DriveToPositionAuton(0.5, -2.5, swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(), swerveSubsystem)
      ),
      new DriveToPlatformAuton(Constants.FORWARD_DIRECTION, swerveSubsystem),
      new BalanceAuton(swerveSubsystem)
    );
  }
}