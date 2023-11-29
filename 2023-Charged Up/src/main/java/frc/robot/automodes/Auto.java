package frc.robot.automodes;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton_commands.sub_commands.ArmAuton;
import frc.robot.auton_commands.sub_commands.BalanceAuton;
import frc.robot.auton_commands.sub_commands.ClawWheelAuton;
import frc.robot.auton_commands.sub_commands.IntakeInAuton;
import frc.robot.auton_commands.sub_commands.IntakeOutAuton;
import frc.robot.auton_commands.sub_commands.ResetOdometry;
import frc.robot.auton_commands.sub_commands.ScoreHighAuton;
import frc.robot.auton_commands.sub_commands.ScoreMidAuton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CTRSwerveSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision.Vision;


import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.Map;




public class Auto {
    public static  CTRSwerveSubsystem m_swerveSubsystem = RobotContainer.m_swerveSubsystem;
    public static  ClawSubsystem m_ClawSubsystem = RobotContainer.m_ClawSubsystem;
    public static  ArmSubsystem m_armSubsystem = RobotContainer.m_armSubsystem;
    public static  IntakeSubsystem m_intakeSubsystem = RobotContainer.m_intakeSubsystem;
    public static  Vision m_visionSubsystem = RobotContainer.m_visionSubsystem;

    public Auto(){
        
    }
    private final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(

        Map.entry("Reset Odometry", new ResetOdometry(m_visionSubsystem, m_swerveSubsystem)),

        Map.entry("Balance", new BalanceAuton(m_swerveSubsystem)),

        //Intake Map
        Map.entry("Low Intake", new IntakeInAuton(m_intakeSubsystem)),
        Map.entry("Low Outtake", new IntakeOutAuton(m_intakeSubsystem)),

        //Arm Outtake Map
        Map.entry("Score High Cone", new ScoreHighAuton(Constants.SCORE_CONE, m_armSubsystem, m_ClawSubsystem)),
        Map.entry("Score High Cube", new ScoreHighAuton(Constants.SCORE_CUBE, m_armSubsystem, m_ClawSubsystem)),
        Map.entry("Score Mid Cone", new ScoreMidAuton(Constants.SCORE_CONE, m_armSubsystem, m_ClawSubsystem)),
        Map.entry("Score Mid Cube", new ScoreMidAuton(Constants.SCORE_CUBE, m_armSubsystem, m_ClawSubsystem)),

        //Arm Intake
        //Preferably don't use, ground intake is more accurate
        Map.entry("Intake Low Cube", 
            new ParallelCommandGroup(
                new ArmAuton(m_armSubsystem, Constants.ARM_LOW_STATE),
                new ClawWheelAuton(m_ClawSubsystem, true) 
            )
        ),
        Map.entry("Reset Arm", new ParallelCommandGroup(
            new ArmAuton(m_armSubsystem, Constants.ARM_MID_STATE), 
            new ClawWheelAuton(m_ClawSubsystem, false)))
        ));

    private SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain()::getPoseMeters,
        RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain()::setPose,
        Constants.AUTO_TRANSLATION_CONSTANTS,
        Constants.AUTO_ROTATION_CONSTANTS,
        RobotContainer.m_swerveSubsystem::setChasisSpeeds,
        eventMap,
        RobotContainer.m_swerveSubsystem
        );

    public Command onePointFiveBalanceMid(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("1.5 Piece Balance", new PathConstraints(5.81, 5)));
    }
    public Command twoPointFiveBalanceOpen(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("2.5 Piece Open H Balance", new PathConstraints(5.81, 5)));
    }
    public Command threePieceBumpLow(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("3 Piece Bump L-L", new PathConstraints(5.81, 5)));
    }
    public Command threePieceOpenHigh(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("3 Piece Open H-L", new PathConstraints(5.81, 5)));
    }
    public Command threePieceOpenLow(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("3 Piece Open L-L", new PathConstraints(5.81, 5)));
    }
    public Command mobilityBalance(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Mobility Balance", new PathConstraints(5.81, 5)));
    }
}
