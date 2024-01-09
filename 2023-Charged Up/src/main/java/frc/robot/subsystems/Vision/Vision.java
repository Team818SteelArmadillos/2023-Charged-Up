package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;

public class Vision extends SubsystemBase{
    Pose2d visionOdometry;
    int activePipeline;
    Pose2d[] lastFiveOutputsVision;
    Pose2d[] lastFiveOutputsDrive;
    int lastFiveOutputsIndex;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public Vision(){
        visionOdometry = new Pose2d();
        LimelightHelpers.setPipelineIndex("", 0);
        activePipeline = 0;

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain().m_kinematics, 
            RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters().getRotation(),
            RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain().getSwervePositions(),
            RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters()
            );
    
    }
    
    @Override
    public void periodic(){
        var pipelineLatencyCapture = LimelightHelpers.getLatency_Capture("");
        var pipelineResult = LimelightHelpers.getBotPose2d("");
        var robotPosition = RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain().getPoseMeters();
        var deltaX = Math.abs(robotPosition.getX() - pipelineResult.getX());
        var deltaY = Math.abs(robotPosition.getY() - pipelineResult.getY());
        var deltaR = Math.abs(robotPosition.getRotation().getDegrees() - pipelineResult.getRotation().getDegrees());

        if(-8.4 < pipelineResult.getY() && pipelineResult.getY()< 8.4 //Y is within field range
        && -4.2 < pipelineResult.getX() && pipelineResult.getX() < 4.2 //X is within field range
        && deltaY < 1 //Vision value isn't complete garbage
        && deltaX < 1 //Vision value isn't complete garbage
        && deltaR < 10 //Vision value isn't complete garbage
        )
        swerveDrivePoseEstimator.addVisionMeasurement(pipelineResult, pipelineLatencyCapture); //Updating odometry model with vision measurements. 
        

        swerveDrivePoseEstimator.update(robotPosition.getRotation(),  RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain().getSwervePositions()); //Updating odometry model with module positions
        RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain().setPose(swerveDrivePoseEstimator.getEstimatedPosition()); //Updating module odomtery based on filtered vision inputs
        Logger.getInstance().recordOutput("Fused Pose", swerveDrivePoseEstimator.getEstimatedPosition());

    }

    public void setAprilTag(){
        LimelightHelpers.setPipelineIndex("", 0);
        activePipeline = 0;
    }

    public void setPieceDetection(){
        LimelightHelpers.setPipelineIndex("", 1);
        activePipeline = 1;
    }

    public void updateVisionOdometry(CTRSwerveDrivetrain drivetrain){
        visionOdometry = LimelightHelpers.getBotPose2d("");
    }

    public Pose2d getVisionOdometry(){
        return visionOdometry;
    }

    public int getActivePipeline(){
        return activePipeline;
    }
}