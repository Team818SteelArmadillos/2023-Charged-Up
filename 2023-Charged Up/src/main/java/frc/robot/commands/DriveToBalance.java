package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveToBalance extends CommandBase {

    private int balance_counter;
    private boolean flip_flag;

    private double m_rotation;

    //private double m_incline;
    private double m_speed;
    private double m_xAxis;
    private double m_yAxis;

    private Translation2d m_translation;
    private boolean m_fieldRelative;
    private boolean m_openLoop;
    
    private SwerveDrivetrain m_swerveDrivetrain;

    private SlewRateLimiter m_xAxisARateLimiter;
    private SlewRateLimiter m_yAxisARateLimiter;
    
    public PIDController DrivePID;

    private Timer timer;

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

    public DriveToBalance(SwerveDrivetrain swerveDrivetrain, double speed, double xAxis, double yAxis, boolean fieldRelative, boolean openLoop) {
        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

        timer = new Timer();
        //m_swerveDrivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;
        //m_incline = m_swerveDrivetrain.getRoll();

        m_speed = speed;
        m_xAxis = xAxis;
        m_yAxis = yAxis;

        m_xAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        m_yAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);

        DrivePID = new PIDController(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);

        DrivePID.setTolerance(Constants.ROTATION_TOLERANCE);

        flip_flag = false;
        
    }

    @Override
    public void initialize() {
        balance_counter = 0;
        timer.reset();
        timer.start();
    }
    

    @Override
    public void execute() {

        double yAxis = 0;
        /* Set variables equal to their respective axis */
        if (m_swerveDrivetrain.getRoll() <= -Constants.MINIMUM_INCLINE_THRESHOLD) {
            if (!flip_flag) {
                m_speed = m_speed * 0.9;
            }
            yAxis = m_yAxis;
            flip_flag = true;
        } else if (m_swerveDrivetrain.getRoll() >= Constants.MINIMUM_INCLINE_THRESHOLD) {
            if (flip_flag) {
                m_speed = m_speed * 0.9;
            }
            yAxis = -m_yAxis;
            flip_flag = false;
        } else {
            balance_counter++;
            m_swerveDrivetrain.holdPosition();
        }
        
        double xAxis = -m_xAxis;
        double rAxis = -DrivePID.calculate(m_swerveDrivetrain.getAngle(), 0);

        /* Square joystick inputs */
        double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
        double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
        double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;

        /* Filter joystick inputs using slew rate limiter */
        double yAxisFiltered = m_yAxisARateLimiter.calculate(yAxisSquared);
        double xAxisFiltered = m_xAxisARateLimiter.calculate(xAxisSquared);

        /* Input variables into drive methods */
        m_translation = new Translation2d(yAxisFiltered, xAxisFiltered).times(Constants.MAX_SPEED * m_speed);
        m_rotation = rAxisSquared * Constants.MAX_ANGULAR_VELOCITY * 0.5;
        m_swerveDrivetrain.drive(m_translation, m_rotation, m_fieldRelative, m_openLoop);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerveDrivetrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return balance_counter >= 20;
    }
}