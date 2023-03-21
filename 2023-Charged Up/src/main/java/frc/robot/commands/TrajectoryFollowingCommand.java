package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class TrajectoryFollowingCommand extends CommandBase {
    
    private SwerveDrivetrain swerveDrivetrain;

    public TrajectoryFollowingCommand(SwerveDrivetrain sub) {
        addRequirements(sub);
        swerveDrivetrain = sub;
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
  
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
          List.of(
            new Translation2d(1,0),
            new Translation2d(1,-1) 
          ),
          new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
          trajectoryConfig
        );
  
      // 3. Define PID Controllers for tracking trajectory
      PIDController xController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
      PIDController yController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AUTO_P_THETA_CONTROLLER, 0, 0, Constants.THETA_CONTROLLER_CONSTRAINTS);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveDrivetrain::getPose,
        Constants.swerveKinematics,
        xController,
        yController,
        thetaController,
        swerveDrivetrain::setModuleStates,
        swerveDrivetrain
      );
      
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