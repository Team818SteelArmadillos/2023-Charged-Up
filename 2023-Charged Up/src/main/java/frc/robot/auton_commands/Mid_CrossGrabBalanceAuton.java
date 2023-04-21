// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auton_commands.sub_commands.ArmAuton;
import frc.robot.auton_commands.sub_commands.BalanceAuton;
import frc.robot.auton_commands.sub_commands.DriveToCenterOfPlatform;
import frc.robot.auton_commands.sub_commands.DriveToGroundIntakeAuton;
import frc.robot.auton_commands.sub_commands.DriveToPlatformAuton;
import frc.robot.auton_commands.sub_commands.DriveToPositionAuton;
import frc.robot.auton_commands.sub_commands.ScoreHighAuton;
import frc.robot.auton_commands.sub_commands.SpeedDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Mid_CrossGrabBalanceAuton extends SequentialCommandGroup {
  public Mid_CrossGrabBalanceAuton(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, CTRSwerveSubsystem swerveSubsystem) {
    Rotation2d currentAngle = swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation();
    addCommands(
      new ScoreHighAuton(Constants.SCORE_CONE, armSubsystem, clawSubsystem),
      new ParallelCommandGroup(
        new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
        new DriveToPlatformAuton(Constants.FORWARD_DIRECTION, currentAngle, swerveSubsystem)
      ),
      new DriveToPositionAuton(6.0, 0, currentAngle, swerveSubsystem),
      new ParallelDeadlineGroup(
        new WaitCommand(3.0),
        new DriveToGroundIntakeAuton(7.8, 0.0, armSubsystem, swerveSubsystem, clawSubsystem)
      ),
      new ParallelCommandGroup(
        new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
        new DriveToPlatformAuton(Constants.BACKWARD_DIRECTION, currentAngle, swerveSubsystem)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.6),
        new SpeedDriveCommand(Constants.BACKWARD_DIRECTION, 0.7, swerveSubsystem)
      ),
        //new DriveToCenterOfPlatform(Constants.BACKWARD_DIRECTION, swerveSubsystem),
      new BalanceAuton(swerveSubsystem)
    );
  }
}
