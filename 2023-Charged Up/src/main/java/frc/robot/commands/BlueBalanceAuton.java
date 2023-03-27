// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BlueBalanceAuton extends SequentialCommandGroup {
  /** Creates a new BlueMiddleAuton. */

  static double[] start = {Units.inchesToMeters(70.78), Units.inchesToMeters(-255.11)};
  
  static double[] coordinates = {Units.inchesToMeters(70.78), Units.inchesToMeters(-172.61)};
  
  public BlueBalanceAuton(ArmSubsystem armSubsystem, 
  SwerveDrivetrain swerveDrivetrain, ClawSubsystem clawSubsystem, LEDSubsystem ledSubsystem) {

    addRequirements(swerveDrivetrain, armSubsystem, clawSubsystem, ledSubsystem);
    swerveDrivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(70.78), Units.inchesToMeters(-255.11), new Rotation2d(0)));
    addCommands(
      new ArmAuton(armSubsystem, 2), //sets arm high
      new WaitCommand(0.5),
      new ParallelCommandGroup(new ClawCommand(clawSubsystem, ledSubsystem), new WaitCommand(0.7)), //Dispenses cone
      new ArmAuton(armSubsystem, 3), //sets arm to neutral position
      //new DriveDistance(m_swerveDrivetrain, 7.0, 0.7, 0.0, 1.0, true, true), //Drives to middle of field
      new WaitCommand(0.8),
      new DriveToRamp(swerveDrivetrain, 0.5, 0.0, 1.0, true, true), //Balances
      new DriveToBalance(swerveDrivetrain, 0.105, 0.0, 1.0, true, true), //Balances
      //new AutoBalanceCommand(m_swerveDrivetrain, m_LEDSubsystem),
      new HoldPosition(swerveDrivetrain)
      );
  }
}

 