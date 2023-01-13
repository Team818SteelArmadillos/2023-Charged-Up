// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleSubsytem extends SubsystemBase {
  /** Creates a new SwerveModuleSubsytem. */
  public SwerveModuleSubsytem() {
    private final WPI_TalonSRX drivemotor;
    private final WPI_TalonSRX turningmotor;

    private final CANCoder m_turnEncoder;
  }

  private final PIDController m_turingPIDController =
    new PIDController(Robot.m_constants.
      , 0, 0);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
