// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands.sub_commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LimeNetwork;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToGroundIntakeAuton extends SequentialCommandGroup {
  /** Creates a new GroundIntakeAuton. */
  public DriveToGroundIntakeAuton(double target_x, double target_y, ArmSubsystem armSubsystem, CTRSwerveSubsystem swerveSubsystem, ClawSubsystem clawSubsystem, LimeNetwork limelight, boolean useLime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmAuton(armSubsystem, Constants.ARM_LOW_STATE),
      new ParallelDeadlineGroup(
        new DriveToPositionAuton(target_x, target_y, new Rotation2d(-target_x, -target_y), swerveSubsystem, limelight, useLime),
        new ClawWheelAuton(clawSubsystem, true)
      ),
      new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE)
    );
  }
}
