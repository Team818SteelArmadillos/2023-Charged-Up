package frc.robot.automodes;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.HashMap;
import java.util.Map;




public class Auto {
    //

    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        
    ));

    private static SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain()::getPoseMeters,
        RobotContainer.m_swerveSubsystem.getCTRSwerveDrivetrain()::setPose,
        Constants.AUTO_TRANSLATION_CONSTANTS,
        Constants.AUTO_ROTATION_CONSTANTS,
        RobotContainer.m_swerveSubsystem::setChasisSpeeds,
        eventMap,
        RobotContainer.m_swerveSubsystem
        );

    public static Command onePointFiveBalanceMid(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("1.5 Piece Balance", new PathConstraints(5.81, 5)));
    }
    public static Command twoPointFiveBalanceOpen(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("2.5 Piece Open H Balance", new PathConstraints(5.81, 5)));
    }
    public static Command threePieceBumpLow(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("3 Piece Bump L-L", new PathConstraints(5.81, 5)));
    }
    public static Command threePieceOpenHigh(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("3 Piece Open H-L", new PathConstraints(5.81, 5)));
    }
    public static Command threePieceOpenLow(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("3 Piece Open L-L", new PathConstraints(5.81, 5)));
    }
    public static Command mobilityBalance(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Mobility Balance", new PathConstraints(5.81, 5)));
    }
}
