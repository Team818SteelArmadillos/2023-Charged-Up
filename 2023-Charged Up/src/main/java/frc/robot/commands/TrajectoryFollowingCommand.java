package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.subsystems.CTRSwerveSubsystem;

public class TrajectoryFollowingCommand extends CommandBase {
    
    private CTRSwerveSubsystem swerveDriveSubsystem;
    private CTRSwerveDrivetrain swerveDrivetrain;

    public double turnSetpoint;
    public String trajectoryJSON;

    public TrajectoryFollowingCommand(CTRSwerveSubsystem sub, CTRSwerveDrivetrain com, String trajectoryJSON) {
      swerveDriveSubsystem = sub;
      swerveDrivetrain = com;
      addRequirements(swerveDriveSubsystem);
      this.trajectoryJSON = trajectoryJSON;
    }

    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.AUTO_MAX_SPEED, 
        Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED)
        .setKinematics(Constants.swerveKinematics);
  
      Trajectory trajectory;
      try {
        trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON));
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }

      // 3. Define PID Controllers for tracking trajectory
      PIDController xController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
      PIDController yController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AUTO_P_THETA_CONTROLLER, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
      HolonomicDriveController driveController = new HolonomicDriveController(xController, yController, thetaController);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, null, null, xController, yController, thetaController, null, swerveDriveSubsystem);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
  }
}