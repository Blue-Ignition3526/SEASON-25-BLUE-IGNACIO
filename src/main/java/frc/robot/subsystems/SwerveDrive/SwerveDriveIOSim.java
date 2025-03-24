package frc.robot.subsystems.SwerveDrive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SwerveDriveConstants;
import lib.BlueShift.control.SpeedAlterator;

public class SwerveDriveIOSim implements SwerveDriveIO {
    // Create all swerve modules
    private final SwerveModuleSim m_frontLeft;
    private final SwerveModuleSim m_frontRight;
    private final SwerveModuleSim m_backLeft;
    private final SwerveModuleSim m_backRight;

    private ChassisSpeeds speeds = new ChassisSpeeds();
    private double heading = 0;
    private double speedsUpdated = Timer.getFPGATimestamp();
    private boolean drivingRobotRelative = false;

    private SpeedAlterator speedAlterator = null;

    public SwerveDriveIOSim() {
        this.m_frontLeft = new SwerveModuleSim("FrontLeft");
        this.m_frontRight = new SwerveModuleSim("FrontRight");
        this.m_backLeft = new SwerveModuleSim("BackLeft");
        this.m_backRight = new SwerveModuleSim("BackRight");
    }

    public void zeroHeading() {
        this.heading = 0;
    }

    public void enableSpeedAlterator(SpeedAlterator alterator) {
        if (this.speedAlterator != alterator) alterator.onEnable();
        if (this.speedAlterator != null) this.speedAlterator.onDisable();
        this.speedAlterator = alterator;
    }

    public void disableSpeedAlterator() {
        if(this.speedAlterator != null) this.speedAlterator.onDisable();
        this.speedAlterator = null;
    }

    /**
     * Get the current ROBOT RELATIVE ChassisSpeeds
     * @return ChassisSpeeds
     */
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) {
            return this.speeds;
        } else {
            return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
        }
    }

    public void xFormation() {
        this.setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        });
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        m_frontLeft.setState(desiredStates[0]);
        m_frontRight.setState(desiredStates[1]);
        m_backLeft.setState(desiredStates[2]);
        m_backRight.setState(desiredStates[3]);
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    public Rotation2d getHeading() {
        // Calculate the new robot heading angle using the angle theta provided 
        return new Rotation2d(this.heading);
    }

    /**
     * Drive the robot using the given speeds (calculate the states for each swerve module and apply it)
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void drive(ChassisSpeeds speeds) {
        if (speedAlterator != null) {
            this.speeds = speedAlterator.alterSpeed(speeds, drivingRobotRelative);
        } else {
            this.speeds = speeds;
        }

        this.speedsUpdated = Timer.getFPGATimestamp();
        SwerveModuleState[] m_moduleStates = SwerveDriveConstants.PhysicalModel.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(m_moduleStates);
    }

    /**
     * Drive the robot relative to the field
     * @param xSpeed X speed in meters per second
     * @param ySpeed Y speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getHeading()));
    }

    /**
     * Drive the robot relative to the field
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot relative to the robot's current position
     * @param xSpeed X speed in meters per second
     * @param ySpeed Y speed in meters per second
     * @param rotSpeed Rotation speed in radians per second
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot relative to the field
     * @param speeds The speeds to drive at (Check `ChassisSpeeds` for more info)
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    public SwerveModuleState[] getModuleRealStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    public SwerveModuleState[] getModuleTargetStates() {
        return getModuleRealStates();
    }

    public void resetDriveEncoders() {
        m_frontLeft.distance = 0;
        m_frontRight.distance = 0;
        m_backLeft.distance = 0;
        m_backRight.distance = 0;
    }

    public void resetTurningEncoders() {
        return;
    }

    public void periodic() {
        this.heading += speeds.omegaRadiansPerSecond * (Timer.getFPGATimestamp() - this.speedsUpdated);

        Logger.recordOutput("SwerveDrive/RobotHeadingRad", this.getHeading().getRadians());
        Logger.recordOutput("SwerveDrive/RobotHeadingDeg", this.getHeading().getDegrees());

        Logger.recordOutput("SwerveDrive/RobotRelative", this.drivingRobotRelative);
        Logger.recordOutput("SwerveDrive/RobotSpeeds", this.getRobotRelativeChassisSpeeds());

        Logger.recordOutput("SwerveDrive/SwerveModuleStates", this.getModuleRealStates());
    }

    public class SwerveModuleSim extends SubsystemBase {
        // Swerve module name
        private final String m_name;
    
        // Current state
        private SwerveModuleState state = new SwerveModuleState();
        private double stateUpdated = Timer.getFPGATimestamp();
        private double distance = 0;
    
        public SwerveModuleSim(String name) {
            m_name = name;
        }
    
        public void stop() {
            state = new SwerveModuleState(0, state.angle);
        }
    
        public void setState(SwerveModuleState state) {
            this.state = state;
            this.stateUpdated = Timer.getFPGATimestamp();
        }
    
        public String getName() {
            return this.m_name;
        }
    
        // Returns the current state
        public SwerveModuleState getState() {
            return state;
        }
    
        // Returns the current state
        public SwerveModuleState getRealState() {
            return state;
        }
    
        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(this.distance, state.angle);
        }
    
        public void periodic() {
            // Calculate the distance the module has traveled since the last update
            this.distance += this.state.speedMetersPerSecond * (Timer.getFPGATimestamp() - this.stateUpdated);

            // Show the target state speed and rotation in radians and degrees on the dashboard
            Logger.recordOutput("SwerveDrive/" + m_name + "/TargetSpeed", state.speedMetersPerSecond);
            Logger.recordOutput("SwerveDrive/" + m_name + "/TargetRotationRad", state.angle.getRadians());
            Logger.recordOutput("SwerveDrive/" + m_name + "/TargetRotationDeg", state.angle.getDegrees());
    
            // Log the state and position of the swerve module
            Logger.recordOutput("SwerveDrive/" + m_name + "/State", state);
            Logger.recordOutput("SwerveDrive/" + m_name + "/Position", getPosition());
        }
    }
}