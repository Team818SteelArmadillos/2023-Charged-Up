package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveToRamp extends CommandBase {

    private int counter;
    private boolean on_ramp_flag;

    private double m_rotation;

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
     * Command for driving to the ramp. The robot will check the incline-angle until it sees that it has tilted up, indicating that we are now on the ramp.
     * The command will end when we are on the ramp and the ramp has flipped.
     * 
     */

    public DriveToRamp(SwerveDrivetrain swerveDrivetrain, double speed, double xAxis, double yAxis, boolean fieldRelative, boolean openLoop) {
        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

        timer = new Timer();
        //m_swerveDrivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;

        m_speed = speed;
        m_xAxis = xAxis;
        m_yAxis = yAxis;

        m_xAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        m_yAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);

        DrivePID = new PIDController(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);

        DrivePID.setTolerance(Constants.ROTATION_TOLERANCE);
        
    }

    @Override
    public void initialize() {
        counter = 0;
        on_ramp_flag = false;
        timer.reset();
        timer.start();
    }
    

    @Override
    public void execute() {
        if (Math.abs(m_swerveDrivetrain.getRoll()) >= Constants.MINIMUM_CHARGE_STATION_ANGLE_THRESH) {
            counter++;
        } else {
            counter = 0;
        }

        if (counter >= 20) {
            on_ramp_flag = true;
        }

        /* Set variables equal to their respective axis */
        double yAxis = -m_yAxis;
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
        return  on_ramp_flag && (Math.abs(m_swerveDrivetrain.getRoll()) <= Constants.MINIMUM_INCLINE_THRESHOLD);
    }
}