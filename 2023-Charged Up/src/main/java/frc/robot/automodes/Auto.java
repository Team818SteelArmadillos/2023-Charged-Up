package frc.robot.automodes;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.auton_commands.sub_commands.BalanceAuton;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


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

    public static Command onePointFiveBalance(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("1.5 Piece Balance", new PathConstraints(5.81, 5)));
    }
    public static Command threePieceOpen(){
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup("3 Piece Open H-L", new PathConstraints(5.81, 5)));
    }
    
    
}
