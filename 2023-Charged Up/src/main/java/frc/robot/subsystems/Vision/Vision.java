package frc.robot.subsystems.Vision;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase{
    Pose2d visionOdometry;
    int activePipeline;
    Pose2d[] lastFiveOutputs;
    int lastFiveOutputsIndex;


    public Vision(){
        visionOdometry = new Pose2d();
        LimelightHelpers.setPipelineIndex("", 0);
        activePipeline = 0;
        Pose2d[] lastFiveOutputs = new Pose2d[5];
        int lastFiveOutputsIndex = 0;
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

        //updating validvalue array
        lastFiveOutputs[lastFiveOutputsIndex] = visionOdometry;
        lastFiveOutputsIndex += 1;

    }
    public Pose2d getVisionOdometry(){
        return visionOdometry;
    }
    public int getActivePipeline(){
        return activePipeline;
    }
    public boolean validValue(){
        double[] X = new double[5];
        double[] Y = new double[5];
        double[] R = new double[5];
        int i = 0;
        for(Pose2d p : lastFiveOutputs){
            X[i] = p.getX();
            Y[i] = p.getY();
            R[i] = p.getRotation().getDegrees();
            i+=1;
        }
        Arrays.sort(X);
        Arrays.sort(Y);
        Arrays.sort(R);
        
        return ((X[4] - X[0] < 0.2) && (Y[4] - Y[0] < 0.2) && (R[4] - R[0] < 0.5));
    }
}
