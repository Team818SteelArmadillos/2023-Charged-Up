package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase{
    Pose2d visionOdometry;
    int activePipeline;


    public Vision(){
        visionOdometry = new Pose2d();
        LimelightHelpers.setPipelineIndex("", 0);
        activePipeline = 0;
    }

    public void setAprilTag(){
        LimelightHelpers.setPipelineIndex("", 0);
        activePipeline = 0;
    }

    public void setCubeDetection(){
        LimelightHelpers.setPipelineIndex("", 1);
        activePipeline = 1;
    }

    public void setConeDetection(){
        LimelightHelpers.setPipelineIndex("", 2);
        activePipeline = 2;
    }

    public void updateVisionOdometry(){
        visionOdometry = LimelightHelpers.getBotPose2d("");
    }
    public Pose2d getVisionOdometry(){
        return visionOdometry;
    }
    public int getActivePipeline(){
        return activePipeline;
    }

}
