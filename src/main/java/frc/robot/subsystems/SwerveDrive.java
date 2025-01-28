package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro.Gyro;
import lib.BlueShift.control.SpeedAlterator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


public class SwerveDrive extends SubsystemBase {
    //* Swerve modules
    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule backLeft;
    public final SwerveModule backRight;

    //* Gyroscope
    public final Gyro gyro;

    //* Odometry
    public final SwerveDriveOdometry odometry;

    //* Speed stats
    private boolean drivingRobotRelative = false;
    private ChassisSpeeds speeds = new ChassisSpeeds();

    SpeedAlterator speedAlterator = null;

    /**
     * Create a new Swerve drivetrain with the provided Swerve Modules and gyroscope
     * @param frontLeft Front Left Swerve Module
     * @param frontRight Front Right Swerve Module
     * @param backLeft Back Left Swerve Module
     * @param backRight Back Right Swerve Module
     * @param gyro Gyroscope
     */
    public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, Gyro gyro) {
        // Store the modules
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        // Store the gyroscope
        this.gyro = gyro;

        // Create odometry with initial data
        this.odometry = new SwerveDriveOdometry(
            Constants.SwerveDrive.PhysicalModel.kDriveKinematics,
            this.getHeading(),
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );

        //! ENCODERS ARE RESET IN EACH MODULE
        //! DO NOT RESET THEM HERE IN THE CONSTRUCTOR

        // Reset gyroscope
        this.gyro.reset();

        // Configure the auto builder
        this.configureAutoBuilder(this);
    }

    /**
     * Configure the PathPlanner auto builder for this swerve drive
     * @param swerveDrive
     */
    public void configureAutoBuilder(Subsystem swerveDrive) {
        RobotConfig config  = null;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();            
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRelativeChassisSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                Constants.SwerveDrive.Autonomous.kTranslatePIDConstants,
                Constants.SwerveDrive.Autonomous.kRotatePIDConstants
            ),
            config,
            () -> {
                if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
                return false;
            },
            swerveDrive
        );
    }

    /**
     * Get the current heading of the robot
     * @return Rotation2d representing the heading of the robot
     */
    public Rotation2d getHeading() {
        return gyro.getHeading();
    }

    /**
     * Zero the heading of the robot (reset the gyro)
     */
    public void zeroHeading() {
        this.gyro.reset();
    }

    /**
     * Get the current pose of the robot
     * @return
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Reset the pose of the robot to (0, 0)
     */
    public void resetPose() {
        resetOdometry(new Pose2d());
    }

    /**
     * Reset the pose of the robot to the provided pose
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(this.getHeading(), getModulePositions(), pose);
    }

    /**
     * If the robot is driving robot relative it will return the speeds directly, otherwise it will return the speeds relative to the field
     * @return
     */
    public ChassisSpeeds getRelativeChassisSpeeds() {
        //! PAST IMPLEMENTATION (RETURNS SPEEDS DIRECTLY)
        if (this.drivingRobotRelative) return this.speeds;
        else return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());

        // TODO: NEW IMPLEMENTATION (RETURNS REAL SPEEDS FROM GYRO)
        // return new ChassisSpeeds(this.gyro.getVelocityX(), this.gyro.getVelocityY(), Math.toRadians(this.gyro.getYawVelocity()));
    }

    /**
     * Get the target module states
     * @return
     */
    public SwerveModuleState[] getModuleTargetStates() {
        return new SwerveModuleState[]{
            frontLeft.getTargetState(),
            frontRight.getTargetState(),
            backLeft.getTargetState(),
            backRight.getTargetState()
        };
    }

    /**
     * Get the real module states
     * @return
     */
    public SwerveModuleState[] getModuleRealStates() {
        return new SwerveModuleState[]{
            frontLeft.getRealState(),
            frontRight.getRealState(),
            backLeft.getRealState(),
            backRight.getRealState()
        };
    }

    /**
     * Get the current module positions
     * @return
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),

            backLeft.getPosition(),
            backRight.getPosition(),
        };
    }

    /**
     * Set the module states
     * @param states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)></b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
        ChassisSpeeds desiredSpeeds = this.speedAlterator != null ? this.speedAlterator.alterSpeed(speeds, drivingRobotRelative) : speeds;
        SwerveModuleState[] m_moduleStates = Constants.SwerveDrive.PhysicalModel.kDriveKinematics.toSwerveModuleStates(desiredSpeeds);
        this.setModuleStates(m_moduleStates);
    }

    public Command enableSpeedAlteratorCommand(SpeedAlterator alterator) {
        return runOnce(() -> {
            alterator.onEnable();
            this.speedAlterator = alterator;
        });
    }

    public Command disableSpeedAlteratorCommand() {
        return runOnce(() -> {
            if(this.speedAlterator != null) this.speedAlterator.onDisable();
            this.speedAlterator = null;
        });
    }


    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getHeading()));
    }

    /**
     * Drive the robot with the provided speeds <b>(FIELD RELATIVE)</b>
     * @param speeds
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

    /**
     * Stop the robot (sets all motors to 0)
     */
    public void stop() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    /**
     * Angle all wheels to point inwards in an X pattern
     */
    public void xFormation() {
        this.frontLeft.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        this.frontRight.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backLeft.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backRight.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    }

    /**
     * Reset the turning encoders of all swerve modules
     */
    public void resetTurningEncoders() {
        this.frontLeft.resetTurningEncoder();
        this.frontRight.resetTurningEncoder();
        this.backLeft.resetTurningEncoder();
        this.backRight.resetTurningEncoder();
    }

    /**
     * Reset the drive encoders of all swerve modules
     */
    public void resetDriveEncoders() {
        this.frontLeft.resetDriveEncoder();
        this.frontRight.resetDriveEncoder();
        this.backLeft.resetDriveEncoder();
        this.backRight.resetDriveEncoder();
    }

    /**
     * Reset all encoders of all swerve modules
     */
    public void resetEncoders() {
        this.resetTurningEncoders();
        this.resetDriveEncoders();
    }

    public Command zeroHeadingCommand() {
        return Commands.runOnce(this::zeroHeading);
    }

    @Override
    public void periodic() {
        // Update odometry
        this.odometry.update(getHeading(), getModulePositions());

        // Log data
        Logger.recordOutput("SwerveDrive/RobotHeadingRad", this.getHeading().getRadians());
        Logger.recordOutput("SwerveDrive/RobotHeadingDeg", this.getHeading().getDegrees());
        
        Logger.recordOutput("SwerveDrive/RobotPose", this.getPose());

        Logger.recordOutput("SwerveDrive/RobotRelative", this.drivingRobotRelative);
        Logger.recordOutput("SwerveDrive/RobotSpeeds", this.getRelativeChassisSpeeds());
        
        Logger.recordOutput("SwerveDrive/ModuleRealStates", this.getModuleRealStates());
        Logger.recordOutput("SwerveDrive/ModuleTargetStates", this.getModuleTargetStates());
    }
}
