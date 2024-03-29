package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrain extends SubsystemBase {

    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveModules;
    public SwerveModulePosition[] m_modulePositions;
    public WPI_Pigeon2 m_gyro;

    /**
     * 
     * Constructor for the entire Swerve Drivetrain
     * 
     * Creates all 4 module instances in addition to gyro and odometry configuration
     * 
     */

    public SwerveDrivetrain() {
        m_gyro = new WPI_Pigeon2(Constants.PIGEON, Constants.CAN_BUS_DRIVE);
        m_gyro.configFactoryDefault();
        /*m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 11000);
        m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 12000);
        m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 13000);
        m_gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 14000); */


        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, 
                             Constants.FRONT_LEFT_OFFSET, 
                             Constants.FRONT_LEFT_AZIMUTH, 
                             Constants.FRONT_LEFT_DRIVE, 
                             Constants.FRONT_LEFT_ENCODER, 
                             Constants.FRONT_LEFT_AZIMUTH_REVERSED,
                             Constants.FRONT_LEFT_DRIVE_REVERSED,
                             Constants.FRONT_LEFT_CANCODER_REVERSED,
                             Constants.FRONT_LEFT_MULTIPLIER),
           new SwerveModule(1,
                             Constants.FRONT_RIGHT_OFFSET,
                             Constants.FRONT_RIGHT_AZIMUTH,
                             Constants.FRONT_RIGHT_DRIVE,
                             Constants.FRONT_RIGHT_ENCODER,
                             Constants.FRONT_RIGHT_AZIMUTH_REVERSED,
                             Constants.FRONT_RIGHT_DRIVE_REVERSED,
                             Constants.FRONT_RIGHT_CANCODER_REVERSED,
                             Constants.FRONT_RIGHT_MULTIPLIER),
            new SwerveModule(2,
                             Constants.BACK_LEFT_OFFSET,
                             Constants.BACK_LEFT_AZIMUTH,
                             Constants.BACK_LEFT_DRIVE,
                             Constants.BACK_LEFT_ENCODER,
                             Constants.BACK_LEFT_AZIMUTH_REVERSED,
                             Constants.BACK_LEFT_DRIVE_REVERSED,
                             Constants.BACK_LEFT_CANCODER_REVERSED,
                             Constants.BACK_LEFT_MULTIPLIER),
            new SwerveModule(3,
                             Constants.BACK_RIGHT_OFFSET,
                             Constants.BACK_RIGHT_AZIMUTH,
                             Constants.BACK_RIGHT_DRIVE,
                             Constants.BACK_RIGHT_ENCODER,
                             Constants.BACK_RIGHT_AZIMUTH_REVERSED,
                             Constants.BACK_RIGHT_DRIVE_REVERSED,
                             Constants.BACK_RIGHT_CANCODER_REVERSED,
                             Constants.BACK_RIGHT_MULTIPLIER)
        };

        SwerveModulePosition m_modulePositions[] = {new SwerveModulePosition(0.0, m_swerveModules[0].getCanCoder()),
            new SwerveModulePosition(0.0, m_swerveModules[1].getCanCoder()),
            new SwerveModulePosition(0.0, m_swerveModules[2].getCanCoder()),
            new SwerveModulePosition(0.0, m_swerveModules[3].getCanCoder())
        };

        this.m_modulePositions = m_modulePositions;
        
        m_swerveOdometry = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), m_modulePositions);

        resetGyro();
    }

    /**
     * 
     * The drive method for the drivetrain
     * 
     * @param translation Translation2d object containing a vector which represents the distance to be traveled in x and y axes
     * @param rotation The holonomic rotation value from the rotation joystick axis
     * @param fieldRelative If the robot is driving field relative or not, should only be false in the case of a brownout
     * @param isOpenLoop Whether or not the robot is driving using open loop control, almost always false
     * 
     */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        SwerveModuleState[] swerveModuleStates =
            Constants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

        for(SwerveModule module : m_swerveModules){
            module.setDesiredState(swerveModuleStates[module.m_moduleNumber], isOpenLoop);
        }

    }

    /**
     * 
     * Sets all 4 modules to the desired states using an array of SwerveModuleStates based on calculations
     * 
     * @param desiredStates An array of all 4 desired states
     * 
     */

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED);
        
        for(SwerveModule mod : m_swerveModules){
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], true); // 
        }
    }

    /**
     * 
     * Sets all 4 modules to the desired states using a ChassisSpeeds object
     * 
     * @param targetSpeeds The target speeds for all modules in ChassisSpeeds form
     * 
     */
    
    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(Constants.swerveKinematics.toSwerveModuleStates(targetSpeeds));
    }

    /**
     * 
     * @return Pose of the robot in meters 
     * 
     */

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public double getRoll() {
        return m_gyro.getRoll() + Constants.PIGEON_ROLL_OFFSET;
    }
    /**
     * 
     * Sets the robot odometry to the current known pose
     * 
     * @param pose Current pose of the robot
     * 
     */

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), m_modulePositions, pose);
    }

    public void resetOdometryWithYaw(Pose2d pose, Rotation2d degrees) {
        m_swerveOdometry.resetPosition(degrees, m_modulePositions, pose);
    }

    /**
     * 
     * Gets the theta angle of the robot
     * 
     * @return Current gyro Yaw value in degrees -180-180
     * 
     */

    public double getAngle() {
        double angle = (m_gyro.getYaw());
        return angle;
    }

    public double getNonContinuousGyro() {
        return getAngle() % 360;
    }

    /**
     * 
     * @return Array of SwerveModuleStates containing the states of all 4 robots
     * 
     */

    public SwerveModulePosition[] getStates(){
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveModules){
            states[mod.m_moduleNumber] = new SwerveModulePosition((mod.getDriveEncoder()/Constants.DRIVE_TICKS_PER_REVOLUTION) * Constants.WHEEL_CIRCUMFERENCE, mod.getState().angle);
        }
        return states;
    }

    /**
     * 
     * Returns the theta angle of the robot as a Rotation2d object
     * 
     * @return Gyro angle as Rotation2d
     * 
     */

    public Rotation2d getYaw() {
        return (Constants.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - (m_gyro.getYaw())%360) 
                                             : Rotation2d.fromDegrees((m_gyro.getYaw())%360);
    }

    /**
     * 
     * Sets the current gyro angle to 0 no matter the robot orientation
     * 
     */

    public void resetGyro() {
        m_gyro.setYaw(0);
    }

    /**
     * 
     * Sets all module positions to 0 no matter their orientation
     * 
     */

    public void zeroModules() {

        for(SwerveModule mod: m_swerveModules) {
            mod.zeroModule();
        }

    }

    /**
     * 
     * Optimizes the holonomic rotation of the robot
     * 
     * @param currentAngle current theta orientation of the robot
     * @param desiredAngle desired theta orientation of the robot
     * @return which direction the robot should turn: true = left, false = right
     * 
     * Note: WILL ALWAYS RETURN A VALUE, EVEN IF CURRENT = DESIRED SO MAKE SURE TO ONLY RUN THIS METHOD WHEN CURRENT != DESIRED
     * 
     */

    public boolean optimizeTurning(double currentAngle, double desiredAngle) {

        boolean isDesiredPositive = desiredAngle > 0;
        boolean isCurrentPositive = currentAngle > 0;

        double m_desiredAngle = Math.abs(desiredAngle);
        double m_currentAngle = Math.abs(currentAngle);

        double absTotal = Math.abs(currentAngle) + Math.abs(desiredAngle);

        if(isDesiredPositive && isCurrentPositive) {
            if(m_desiredAngle > m_currentAngle) {
                return true;
            } else {
                return false;
            }
        } else if(!isDesiredPositive && !isCurrentPositive) {
            if(m_desiredAngle > m_currentAngle) {
                return true;
            } else {
                return false;
            }
        } else {
            if(absTotal > 180) {
                return false;
            } else {
                return true;
            }
        }
        

    }

    /**
     * 
     * @return array of SwerveModules
     * 
     */

    public SwerveModule[] getModules() {
        return m_swerveModules;
    }

    /**
     * 
     * @param yaw yaw value to set the gyro to
     * 
     */

    public void setGyro(double yaw) {
        double yawMod;
        if(yaw < 0) {
            yawMod = 360 - yaw;
        } else if(yaw > 0) {
            yawMod = yaw;
        } else {
            yawMod = yaw;
        }
        m_gyro.setYaw(yawMod);
    }

    /**
     * 
     * Updates odometry with current theta angle and module states
     * 
     * Pushes module cancoder and integrated encoder values, module velocities, and gyro angle to SmartDashboard
     * 
     */

     public void stopModules() {
        for (SwerveModule module : m_swerveModules) {
            module.stop();
        }
     }

     public void holdPosition() {
        for (SwerveModule module : m_swerveModules) {
            module.hold();
        }
     }

    @Override
    public void periodic(){
        
        m_swerveOdometry.update(getYaw(), getStates());

        SmartDashboard.putNumber("Roll", getRoll());

        // SmartDashboard.putNumber("Font Left Offset", m_swerveModules[0].getCandcoderAbsPos());
        // SmartDashboard.putNumber("Font Right Offset", m_swerveModules[1].getCandcoderAbsPos());
        // SmartDashboard.putNumber("Back Left Offset", m_swerveModules[2].getCandcoderAbsPos());
        // SmartDashboard.putNumber("Back Right Offset", m_swerveModules[3].getCandcoderAbsPos());

        // SmartDashboard.putNumber("Font Left Voltage", m_swerveModules[0].getAzimuthVoltage());
        // SmartDashboard.putNumber("Font Right Votlage", m_swerveModules[1].getAzimuthVoltage());
        // SmartDashboard.putNumber("Back Left Voltage", m_swerveModules[2].getAzimuthVoltage());
        // SmartDashboard.putNumber("Back Right Votlage", m_swerveModules[3].getAzimuthVoltage());

        Logger.getInstance().recordOutput("Robot Pose", getPose());
        SmartDashboard.putNumber("pose x", getPose().getX());
        SmartDashboard.putNumber("pose y", getPose().getY());
        SmartDashboard.putNumber("pose rot", getPose().getRotation().getDegrees());

    }

    
}
