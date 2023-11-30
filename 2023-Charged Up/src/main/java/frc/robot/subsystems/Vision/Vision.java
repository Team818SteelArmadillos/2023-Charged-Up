package frc.robot.subsystems.Vision;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.OdometryMonitor;

public class Vision extends SubsystemBase{
    Pose2d visionOdometry;
    int activePipeline;
    Pose2d[] lastFiveOutputs;
    int lastFiveOutputsIndex;


    public Vision(){
        visionOdometry = new Pose2d();
        LimelightHelpers.setPipelineIndex("", 0);
        activePipeline = 0;
        lastFiveOutputs = new Pose2d[5];
        for(int i=0; i<=4 ; i++){
            lastFiveOutputs[i] = LimelightHelpers.getBotPose2d("");
        }
        lastFiveOutputsIndex = 0;
    }

    public void setAprilTag(){
        LimelightHelpers.setPipelineIndex("", 0);
        activePipeline = 0;
    }

    public void setPieceDetection(){
        LimelightHelpers.setPipelineIndex("", 1);
        activePipeline = 1;
    }

    public void updateVisionOdometry(){
        visionOdometry = LimelightHelpers.getBotPose2d("");

        //updating validvalue array
        if(lastFiveOutputsIndex == 5){
            lastFiveOutputsIndex = 0;
        }
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
        
        return ((Math.abs(X[4] - X[0]) < 0.2) && (Math.abs(Y[4] - Y[0]) < 0.2) && (Math.abs(R[4] - R[0]) < 0.5));
    }
}
