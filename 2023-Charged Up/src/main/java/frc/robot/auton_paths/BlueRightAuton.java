// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton_paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auton_commands.ArmAuton;
import frc.robot.auton_commands.BalanceAuton;
import frc.robot.auton_commands.ClawWheelAuton;
import frc.robot.auton_commands.DriveToPositionAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueRightAuton extends SequentialCommandGroup {
  
  
  
  /** Creates a new BlueRightAuton. */
  public BlueRightAuton(
    ArmAuton armAuton,
    ArmSubsystem armSubsystem,
    ClawWheelAuton ClawAuto,
    CTRSwerveSubsystem driveSubsystem
  ) 
  
  {
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //armAuton(armSubsystem, 0),
      new DriveToPositionAuton(0.652, -1.6358, new Rotation2d(), driveSubsystem)

    );
  
  
  }
}
