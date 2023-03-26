package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveDrive extends CommandBase {

    private double m_rotation;
    private Translation2d m_translation;
    private boolean m_fieldRelative;
    private boolean m_openLoop;
    
    private SwerveDrivetrain m_swerveDrivetrain;
    private XboxController m_driverController;
    private int m_driveAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;
    private double m_speedFactor;

    private SlewRateLimiter m_xAxisARateLimiter;
    private SlewRateLimiter m_yAxisARateLimiter;

    private double targetAngle;
    
    public PIDController DrivePID;

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

    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, XboxController driverController, int driveAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

        m_driverController = driverController;
        m_driveAxis = driveAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;
        targetAngle = 0;
        m_speedFactor = 1.0;

        m_xAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        m_yAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        DrivePID = new PIDController(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);
       // DrivePID.setTolerance(1);
        

    }

    @Override
    public void execute() {

        if (m_driverController.getXButton()) {
            m_swerveDrivetrain.resetGyro();
            targetAngle = m_swerveDrivetrain.getAngle();
        }

        if (m_driverController.getYButton()) {
            m_swerveDrivetrain.resetModulesToAbsolute();
            Timer.delay(0.1);
        }

        if (OI.getDriver().getRightTriggerAxis() >= 0.5) {
            m_speedFactor = 0.5;
        } else {
            m_speedFactor = 1.0;
        }

        /* Set variables equal to their respective axis */
        double yAxis = m_driverController.getRawAxis(m_driveAxis);
        double xAxis = m_driverController.getRawAxis(m_strafeAxis);
        double rAxis = -m_driverController.getRawAxis(m_rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.STICK_DEADBAND) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.STICK_DEADBAND) ? 0 : xAxis;

        if ((Math.abs(rAxis) < Constants.STICK_DEADBAND)) {
            rAxis = -DrivePID.calculate(m_swerveDrivetrain.getAngle(), targetAngle);
        } else {
            targetAngle = m_swerveDrivetrain.getAngle();
        }
        

        /* Square joystick inputs */
        double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
        double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
        double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;

        /* Filter joystick inputs using slew rate limiter */
        double yAxisFiltered = m_yAxisARateLimiter.calculate(yAxisSquared);
        double xAxisFiltered = m_xAxisARateLimiter.calculate(xAxisSquared);

        /* Input variables into drive methods */
        m_translation = new Translation2d(yAxisFiltered, xAxisFiltered).times(Constants.MAX_SPEED * m_speedFactor);
        m_rotation = rAxisSquared * Constants.MAX_ANGULAR_VELOCITY * 0.5; // if 
        m_swerveDrivetrain.drive(m_translation, m_rotation, m_fieldRelative, m_openLoop);
        //SmartDashboard.putNumber("Input Angle", m_translation.getAngle().getDegrees());
        //SmartDashboard.putNumber("Rotation Angle", m_swerveDrivetrain.getAngle());
    }
}