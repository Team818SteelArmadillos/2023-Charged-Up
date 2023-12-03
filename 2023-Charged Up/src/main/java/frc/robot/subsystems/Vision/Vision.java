package frc.robot.subsystems.Vision;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.commands.OdometryMonitor;

public class Vision extends SubsystemBase{
    Pose2d visionOdometry;
    int activePipeline;
    Pose2d[] lastFiveOutputsVision;
    Pose2d[] lastFiveOutputsDrive;
    int lastFiveOutputsIndex;


    public Vision(){
        visionOdometry = new Pose2d();
        LimelightHelpers.setPipelineIndex("", 0);
        activePipeline = 0;
        lastFiveOutputsVision = new Pose2d[5];
        for(int i=0; i<=4 ; i++){
            lastFiveOutputsVision[i] = LimelightHelpers.getBotPose2d("");
        }
        for(int i=0; i<4; i++){
            lastFiveOutputsDrive[i] = new Pose2d(0,0, new Rotation2d(0));
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

    public void updateVisionOdometry(CTRSwerveDrivetrain drivetrain){
        visionOdometry = LimelightHelpers.getBotPose2d("");

        //updating validvalue array
        if(lastFiveOutputsIndex == 5){
            lastFiveOutputsIndex = 0;
        }
        lastFiveOutputsVision[lastFiveOutputsIndex] = visionOdometry;
        lastFiveOutputsDrive[lastFiveOutputsIndex] = drivetrain.getPoseMeters();
        lastFiveOutputsIndex += 1;
    }

    public Pose2d getVisionOdometry(){
        return visionOdometry;
    }

    public int getActivePipeline(){
        return activePipeline;
    }

    public boolean validValue(){
        double deltax = 0;
        double deltay = 0;
        double deltaa = 0;
        
        for(var i = 0; i < 5; i++){
            deltax = deltax + Math.abs(lastFiveOutputsVision[i].getX() - lastFiveOutputsDrive[i].getX());
            deltay = deltay + Math.abs(lastFiveOutputsVision[i].getY() - lastFiveOutputsDrive[i].getY());
            deltaa = deltaa + Math.abs(lastFiveOutputsVision[i].getRotation().getDegrees() - lastFiveOutputsDrive[i].getRotation().getDegrees());
        }
        
        return (deltax < 0.2 && deltay < 0.2 && deltaa < 5);
    }
}
