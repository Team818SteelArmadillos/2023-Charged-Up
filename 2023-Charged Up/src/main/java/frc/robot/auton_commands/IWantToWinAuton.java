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
import frc.robot.auton_commands.sub_commands.DriveToGroundIntakeAuton;
import frc.robot.auton_commands.sub_commands.DriveToPositionAuton;
import frc.robot.auton_commands.sub_commands.ScoreHighAuton;
import frc.robot.commands.ClawModeToggleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IWantToWinAuton extends SequentialCommandGroup {
  /** Creates a new IWantToWinAuton. */
  public IWantToWinAuton(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, CTRSwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Rotation2d rotation = swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation();


    addCommands(
      // score
      new ScoreHighAuton(armSubsystem, clawSubsystem),
      new DriveToGroundIntakeAuton(4.8, -0.4, armSubsystem, swerveSubsystem, clawSubsystem)


      // new ParallelCommandGroup(// ready arm while driving
      //   new ArmAuton(armSubsystem, Constants.ARM_LOW_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem) 
      // ),
      // new ClawModeToggleCommand(clawSubsystem),
      // new WaitCommand(0.2),
      // // pick up cone 1
      // new ParallelCommandGroup(
      //   new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem)
      // ),
      // // score 1
      // new ParallelCommandGroup(
      //   new ArmAuton(armSubsystem, Constants.ARM_MID_STATE),
      //   new WaitCommand(0.2),
      //   new ClawModeToggleCommand(clawSubsystem),
      //   new WaitCommand(0.2)
      // )
      

      // // drive to cone 2
      // new ParallelCommandGroup(// retract arm while driving
      //   new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem)
      // ),
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // new ParallelCommandGroup(// ready arm while driving
      //   new ArmAuton(armSubsystem, Constants.ARM_LOW_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem), 
      //   new ClawWheelAuton(5, clawSubsystem, true)
      // ),
      // // pick up cone 2
      // new ClawModeToggleCommand(clawSubsystem),
      // new WaitCommand(0.3),
      // new ParallelDeadlineGroup(
      //   new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem)
      // ),
      // // drive
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // // score 2
      // new ArmAuton(armSubsystem, Constants.ARM_HIGH_STATE),
      // new WaitCommand(0.5),
      // new ClawModeToggleCommand(clawSubsystem),
      // new WaitCommand(0.3),




      // // drive to cone 3
      // new ParallelCommandGroup(// retract arm while driving
      //   new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem)
      // ),
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // new ParallelCommandGroup(// ready arm while driving
      //   new ArmAuton(armSubsystem, Constants.ARM_LOW_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem), 
      //   new ClawWheelAuton(5, clawSubsystem, true)
      // ),
      // // pick up cone 3
      // new ClawModeToggleCommand(clawSubsystem),
      // new WaitCommand(0.3),
      // new ParallelDeadlineGroup(
      //   new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem)
      // ),
      // // drive
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // // score 3
      // new ArmAuton(armSubsystem, Constants.ARM_HIGH_STATE),
      // new WaitCommand(0.5),
      // new ClawModeToggleCommand(clawSubsystem),
      // new WaitCommand(0.3),


      
      // // drive to cone 4
      // new ParallelCommandGroup(// retract arm while driving
      //   new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem)
      // ),
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // new ParallelCommandGroup(// ready arm while driving
      //   new ArmAuton(armSubsystem, Constants.ARM_LOW_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem), 
      //   new ClawWheelAuton(5, clawSubsystem, true)
      // ),
      // // pick up cone 4
      // new ClawModeToggleCommand(clawSubsystem),
      // new WaitCommand(0.3),
      // new ParallelDeadlineGroup(
      //   new ArmAuton(armSubsystem, Constants.ARM_NEUTRAL_STATE),
      //   new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem)
      // ),
      // // drive
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // // score 4
      // new ArmAuton(armSubsystem, Constants.ARM_HIGH_STATE),
      // new WaitCommand(0.5),
      // new ClawModeToggleCommand(clawSubsystem),
      // new WaitCommand(0.3),


      // prep for teleop
      // new DriveToPositionAuton(0, 0, new Rotation2d(), swerveSubsystem),
      // new BalanceAuton(swerveSubsystem)
    );
  }
}
