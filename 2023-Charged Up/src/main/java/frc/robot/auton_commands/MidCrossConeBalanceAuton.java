// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton_commands.sub_commands.BalanceAuton;
import frc.robot.auton_commands.sub_commands.DriveToGroundIntakeAuton;
import frc.robot.auton_commands.sub_commands.DriveToPositionAuton;
import frc.robot.auton_commands.sub_commands.ScoreHighAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LimeNetwork;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidCrossConeBalanceAuton extends SequentialCommandGroup {
  /** Creates a new IWantToWinAuton. */
  public LimeNetwork m_limelight = new LimeNetwork();

  public MidCrossConeBalanceAuton(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, CTRSwerveSubsystem swerveSubsystem, LimeNetwork limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreHighAuton(armSubsystem, clawSubsystem),
      new DriveToPositionAuton(6.0, 0, swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(), swerveSubsystem, limelight, false),
      new DriveToGroundIntakeAuton(7, 0, armSubsystem, swerveSubsystem, clawSubsystem, limelight, true),
      new DriveToPositionAuton(3.0, 0, swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(), swerveSubsystem, limelight, true),
      new BalanceAuton(swerveSubsystem)
    );
  }
}
