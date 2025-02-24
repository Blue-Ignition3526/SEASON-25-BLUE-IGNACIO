package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants.SwerveModuleConstants;
import lib.BlueShift.constants.SwerveModuleOptions;
import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {
    // * Options for the module
    public final SwerveModuleOptions options;

    // * Motors
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    // * Configs
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;

    // * PID Controller for turning
    public final SparkClosedLoopController turnPID;

    // * Absolute encoder
    private final CANcoder absoluteEncoder;
    private final StatusSignal<Angle> absolutePositionSignal;

    // * Target state
    private SwerveModuleState targetState = new SwerveModuleState();

    // * Alerts
    private final Alert alert_cancoderUnreachable;
    private final Alert alert_driveMotorUnreachable;
    private final Alert alert_turnMotorUnreachable;
    private final Alert alert_turnEncodersOutOfSync;

    // * Device check notifier
    private final Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

    /**
     * Create a new swerve module with the provided options
     * @param options
     */
    public SwerveModule(SwerveModuleOptions options) {
        // * Store the options
        this.options = options;

        // * Create Drive motor and configure it
        this.driveMotor = new SparkMax(options.driveMotorID, MotorType.kBrushless);
        this.driveConfig = new SparkMaxConfig();
        this.driveConfig
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(SwerveModuleConstants.kDriveMotorRampRate)
            .closedLoopRampRate(SwerveModuleConstants.kDriveMotorRampRate)
            .smartCurrentLimit(SwerveModuleConstants.kDriveMotorCurrentLimit)
            .voltageCompensation(12);

        // Configure the drive encoder
        this.driveConfig.encoder
            .positionConversionFactor(Constants.SwerveDriveConstants.PhysicalModel.kDriveEncoder_RotationToMeter)
            .velocityConversionFactor(Constants.SwerveDriveConstants.PhysicalModel.kDriveEncoder_RPMToMeterPerSecond)
            // TODO: Verify these options
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        // Apply config to drive motor
        this.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // * Create Turn motor and configure it
        this.turnMotor = new SparkMax(options.turningMotorID, MotorType.kBrushless);
        this.turnConfig = new SparkMaxConfig();
        this.turnConfig
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(SwerveModuleConstants.kDriveMotorRampRate)
            .closedLoopRampRate(SwerveModuleConstants.kDriveMotorRampRate)
            .smartCurrentLimit(SwerveModuleConstants.kDriveMotorCurrentLimit)
            .voltageCompensation(12)
            .inverted(true);

        // Configure the turning encoder
        this.turnConfig.encoder
            .positionConversionFactor(Constants.SwerveDriveConstants.PhysicalModel.kTurningEncoder_Rotation)
            .velocityConversionFactor(Constants.SwerveDriveConstants.PhysicalModel.kTurningEncoder_RPS)
            // TODO: Verify these options
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        // Configure closed loop controller
        this.turnConfig.closedLoop
            .p(Constants.SwerveDriveConstants.SwerveModuleConstants.kTurningPIDConstants.kP)
            .i(Constants.SwerveDriveConstants.SwerveModuleConstants.kTurningPIDConstants.kI)
            .d(Constants.SwerveDriveConstants.SwerveModuleConstants.kTurningPIDConstants.kD)
            .positionWrappingInputRange(0, 1)
            .positionWrappingEnabled(true);
        
        // Apply config to turn motor
        this.turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // * Get the PID controller for the turning motor
        this.turnPID = turnMotor.getClosedLoopController();

        // * Absolute encoder
        this.absoluteEncoder = new CANcoder(options.absoluteEncoderDevice.getDeviceID(), options.absoluteEncoderDevice.getCanbus());
        this.absolutePositionSignal = absoluteEncoder.getAbsolutePosition(false);

        // * Alerts
        this.alert_cancoderUnreachable = new Alert(options.name + " Swerve Module AbsEnc unreachable", AlertType.kError);
        this.alert_driveMotorUnreachable = new Alert(options.name + " Swerve Module DriveMot unreachable", AlertType.kError);
        this.alert_turnMotorUnreachable = new Alert(options.name + " Swerve Module TurnMot unreachable", AlertType.kError);
        this.alert_turnEncodersOutOfSync = new Alert(options.name + " Swerve Module Encoders out of sync", AlertType.kWarning);

        // * Reset encoders
        // Wait for absolute encoder to become available and return the boot-up position
        System.out.println("Waiting for " + options.name + " Swerve Module absolute encoder to become available...");
        StatusCode absoluteEncoderStatusCode = this.absolutePositionSignal.waitForUpdate(20).getStatus();

        if (absoluteEncoderStatusCode.isOK()) {
            System.out.println(options.name + " Swerve Module absolute encoder is available, resetting position...");
            resetTurningEncoder();
        } else {
            DriverStation.reportError("Failed to get " + options.name + " Swerve Module absolute encoder position. Status: " + absoluteEncoderStatusCode, false);
            System.out.println("Please check " + options.name + " Swerve Module absolute encoder connection.");
            System.out.println("If problem persists please align the wheels manually and restart the robot.");
            System.out.println("Setting current position as 0.");
            this.turnMotor.getEncoder().setPosition(0);
        }

        // * Reset the Drive encoder
        resetDriveEncoder();

        // * Start the device check notifier
        deviceCheckNotifier.startPeriodic(10);
    }

    /**
     * Check if the devices are reachable
     */
    private final void deviceCheck() {
        try {
            driveMotor.getFirmwareVersion();
            alert_driveMotorUnreachable.set(false);
        } catch (Exception e) {
            alert_driveMotorUnreachable.set(true);
            DriverStation.reportError(options.name +  " drive motor is unreachable", false);
        }

        try {
            turnMotor.getFirmwareVersion();
            alert_turnMotorUnreachable.set(false);
        } catch (Exception e) {
            alert_turnMotorUnreachable.set(true);
            DriverStation.reportError(options.name +  " turn motor is unreachable", false);
        }

        if (absolutePositionSignal.getStatus().isOK()) {
            alert_cancoderUnreachable.set(false);
        } else {
            alert_cancoderUnreachable.set(true);
            DriverStation.reportError(options.name +  " absolute encoder is unreachable", false);
        }

        double turnEncErr = getAngle().minus(getAbsoluteEncoderPosition()).in(Degree);
        if (Math.abs(turnEncErr) > 5) {
            alert_turnEncodersOutOfSync.set(true);
            DriverStation.reportError(options.name +  " turning encoders are out of sync (" + String.valueOf(turnEncErr) + "°)", false);
        } else {
            alert_turnEncodersOutOfSync.set(false);
        }
    }

    /**
     * Get the absolute encoder turn position
     * @return
     */
    public Angle getAbsoluteEncoderPosition() {
        return absolutePositionSignal.refresh().getValue();
    }

    /**
     * Reset the drive encoder (set the position to 0)
     */
    public void resetDriveEncoder() {
        this.driveMotor.getEncoder().setPosition(0);
    }

    /**
     * Reset the turning encoder (set the position to the absolute encoder's position)
     */
    public void resetTurningEncoder() {
        this.turnMotor.getEncoder().setPosition(getAbsoluteEncoderPosition().in(Rotations));
    }
    
    /**
     * Reset the drive and turning encoders
     */
    public void resetEncoders() {
        resetDriveEncoder();
        resetTurningEncoder();
    }

    /**
     * Get the current angle of the module
     * @return
     */
    public Angle getAngle() {
        return Rotations.of(this.turnMotor.getEncoder().getPosition());
    }

    /**
     * Set the target state of the module
     * @param state The target state
     */
    public void setTargetState(SwerveModuleState state) {
        setTargetState(state, false);
    }

    /**
     * Set the target state of the module
     * @param state The target state
     * @param force If true, the module will ignore the current speed and turn to the target angle
     */
    public void setTargetState(SwerveModuleState state, boolean force) {
        

        if (Math.abs(state.speedMetersPerSecond) < Double.MIN_VALUE || force) {
            stop();
            return;
        }

        // Optimize the angle
        state.optimize(Rotation2d.fromRotations(getAngle().in(Rotations)));
        
        // Scale the target state for smoother movement
        state.cosineScale(Rotation2d.fromRotations(getAngle().in(Rotations)));

        // Set the target state for safekeeping
        this.targetState = state;

        // Set the motor speeds
        SmartDashboard.putNumber("Debugging/TF3/drive"+this.options.name, state.speedMetersPerSecond);
        SmartDashboard.putNumber("Debugging/TF3/maxSpeed"+this.options.name, Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        SmartDashboard.putNumber("Debugging/TF3/rot"+this.options.name, state.angle.getRotations());


        driveMotor.set(state.speedMetersPerSecond / Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        turnPID.setReference(state.angle.getRotations(), ControlType.kPosition);
    }

    /**
     * Stop the module (set the speed of the motors to 0)
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Get the target state of the module
     * @return
     */
    public SwerveModuleState getTargetState() {
        return this.targetState;
    }

    /**
     * Get the real state of the module
     * @return
     */
    public SwerveModuleState getRealState() {
        return new SwerveModuleState(
            this.driveMotor.getEncoder().getVelocity() * Math.PI,
            Rotation2d.fromRotations(this.getAngle().in(Rotations))
        );
    }

    /**
     * Get the position of the module
     * @return
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveMotor.getEncoder().getPosition() * Math.PI,
            Rotation2d.fromRotations(this.getAngle().in(Rotations))
        );
    }

    public void periodic() {
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/MotEncoderDeg", this.getAngle().in(Radians));
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/AbsEncoderDeg", this.getAbsoluteEncoderPosition().in(Radians));
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/RealState", this.getRealState());
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/TargetState", this.getTargetState());
    }
}
