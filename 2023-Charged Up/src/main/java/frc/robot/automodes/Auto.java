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
    private CTRSwerveDrivetrain m_drivetrain;
    
    m_drivetrain = drivetrain.getCTRSwerveDrivetrain();
    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        
    ));
    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    CTRSwerveDrivetrain.m_odometry::getPoseMeters,
    RobotContainer.m_swerveSubsystem::resetOdometry,
    Constants.AUTO_TRANSLATION_CONSTANTS,
    Constants.AUTO_ROTATION_CONSTANTS,
    RobotContainer.m_swerveSubsystem::setChasisSpeeds,
    eventMap,
    true,
    RobotContainer.m_swerveSubsystem);    
    
}
