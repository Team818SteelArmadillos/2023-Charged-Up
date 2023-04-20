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
import frc.robot.auton_commands.sub_commands.ClawWheelAuton;
import frc.robot.auton_commands.sub_commands.DriveToGroundIntakeAuton;
import frc.robot.auton_commands.sub_commands.DriveToPlatformAuton;
import frc.robot.auton_commands.sub_commands.DriveToPositionAuton;
import frc.robot.auton_commands.sub_commands.IntakeInAuton;
import frc.robot.auton_commands.sub_commands.ScoreHighAuton;
import frc.robot.auton_commands.sub_commands.SpeedDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Clear_ScoreTwoAndBalanceAuton extends SequentialCommandGroup {
  /** Creates a new IWantToWinAuton. */
  public Clear_ScoreTwoAndBalanceAuton(String alliance, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, CTRSwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double first_cube_x = 0;
    double first_cube_y = 0;
    double second_cube_x = 0;
    double second_cube_y = 0;
    int alliance_y_direction = 1;

    if (alliance.equalsIgnoreCase("Blue")) {
      first_cube_x = Constants.FIRST_CUBE_X_POS_BLUE;
      first_cube_y = Constants.FIRST_CUBE_Y_POS_BLUE;
      second_cube_x = Constants.SECOND_CUBE_X_POS_BLUE;
      second_cube_y = Constants.SECOND_CUBE_Y_POS_BLUE;
      alliance_y_direction = -1;
    } else if (alliance.equalsIgnoreCase("Red")) {
      first_cube_x = Constants.FIRST_CUBE_X_POS_RED;
      first_cube_y = Constants.FIRST_CUBE_Y_POS_RED;
      second_cube_x = Constants.SECOND_CUBE_X_POS_RED;
      second_cube_y = Constants.SECOND_CUBE_Y_POS_RED;
      alliance_y_direction = 1;
    } else {

    }

    addCommands(
      new ScoreHighAuton(Constants.SCORE_CONE, armSubsystem, clawSubsystem),
      new ParallelDeadlineGroup(
        new ArmAuton(armSubsystem, Constants.ARM_LOW_STATE),
        new SpeedDriveCommand(Constants.FORWARD_DIRECTION, 0.4, swerveSubsystem)
      ),
      new DriveToGroundIntakeAuton(first_cube_x, alliance_y_direction * first_cube_y, armSubsystem, swerveSubsystem, clawSubsystem),
      new ParallelCommandGroup(
        new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
        new DriveToPositionAuton(0.1, alliance_y_direction * 0.67, swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(), swerveSubsystem)
      ),
      new ScoreHighAuton(Constants.SCORE_CUBE, armSubsystem, clawSubsystem),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25), 
        new ClawWheelAuton(clawSubsystem, false)
      ),
      new ParallelCommandGroup(
        new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
        new DriveToPositionAuton(4.0, alliance_y_direction * 0.2, swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(), swerveSubsystem)
      ),
      new ParallelDeadlineGroup(
        new DriveToPositionAuton(Constants.SIDE_INTAKE_DIRECTION, second_cube_x, alliance_y_direction * second_cube_y, swerveSubsystem),
        new IntakeInAuton(intakeSubsystem)
      ),
      new DriveToPositionAuton(6.0, alliance_y_direction * 2.5, swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(), swerveSubsystem),
      new DriveToPlatformAuton(Constants.BACKWARD_DIRECTION, swerveSubsystem),
      new BalanceAuton(swerveSubsystem)
    );
  }
}
