package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveDistance extends CommandBase {

    private double m_rotation;
    private Translation2d m_translation;
    private boolean m_fieldRelative;
    private boolean m_openLoop;
    
    private SwerveDrivetrain m_swerveDrivetrain;
    private int m_driveAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;

    private SlewRateLimiter m_xAxisARateLimiter;
    private SlewRateLimiter m_yAxisARateLimiter;

    private double targetAngle;
    private Pose2d targetPose;
    
    public PIDController DrivePID;
    public PIDController DistanceXPID;
    public PIDController DistanceYPID;

    /**
     * 
     * Command for driver-controlled driving
     * 
     * @param swerveDrivetrain drivetrain instance
     * @param driverController driver XboxController object
     * @param driveAxis corresponding integer for the y-axis translation joystick axis
     * @param strafeAxis corresponding integer for the x-axis translation joystick axis
     * @param rotationAxis corresponding integer for the rotation joystick axis
     * @param fieldRelative whether or not driving is field relative
     * @param openLoop whether or not driving is open loop
     * 
     */

    public DriveDistance(SwerveDrivetrain swerveDrivetrain, Pose2d targetPose, boolean fieldRelative, boolean openLoop) {
        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);


        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;
        targetAngle = 0;

        m_xAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        m_yAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        DrivePID = new PIDController(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);
        DistanceXPID = new PIDController(Constants.csP, Constants.csI, Constants.csD);
        DistanceYPID = new PIDController(Constants.csP, Constants.csI, Constants.csD);
        DistanceXPID.setTolerance(Constants.csTolerance);
        DistanceYPID.setTolerance(Constants.csTolerance);
        

    }

    @Override
    public void execute() {

        

        /* Set variables equal to their respective axis */
        double yAxis = DistanceYPID.calculate(m_swerveDrivetrain.getPose().getY(), targetPose.getY());
        double xAxis = DistanceXPID.calculate(m_swerveDrivetrain.getPose().getX(), targetPose.getX());
        double rAxis = 0;
        
        /* Deadbands */

        targetAngle = targetAngle + (rAxis * 5);

        rAxis = -DrivePID.calculate(m_swerveDrivetrain.getAngle(), targetAngle);

        /* Square joystick inputs */
        double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
        double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
        double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;

        /* Filter joystick inputs using slew rate limiter */
        double yAxisFiltered = m_yAxisARateLimiter.calculate(yAxisSquared);
        double xAxisFiltered = m_xAxisARateLimiter.calculate(xAxisSquared);

        /* Input variables into drive methods */
        m_translation = new Translation2d(yAxisFiltered, xAxisFiltered).times(Constants.MAX_SPEED);
        m_rotation = rAxisSquared * Constants.MAX_ANGULAR_VELOCITY * 0.5; // if 
        m_swerveDrivetrain.drive(m_translation, m_rotation, m_fieldRelative, m_openLoop);
        SmartDashboard.putNumber("Input Angle", m_translation.getAngle().getDegrees());
        SmartDashboard.putNumber("Rotation Angle", m_swerveDrivetrain.getAngle());
    }
    

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return DistanceXPID.atSetpoint() && DistanceYPID.atSetpoint();
        //return pivotingArmSubsystem.PID.atSetpoint();
    }
}