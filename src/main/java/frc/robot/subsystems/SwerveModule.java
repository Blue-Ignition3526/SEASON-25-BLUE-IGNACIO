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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.constants.SwerveModuleOptions;
import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {
    //* Options for the module
    public final SwerveModuleOptions options;

    //* Motors
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    // * Configs
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;

    //* PID Controller for turning
    public final SparkClosedLoopController turnPID;

    //* Absolute encoder
    private final CANcoder absoluteEncoder;
    private final StatusSignal<Angle> absolutePositionSignal;

    //* Target state
    private SwerveModuleState targetState = new SwerveModuleState();

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
        this.driveConfig.inverted(options.driveMotorInverted);

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
        this.turnConfig.inverted(options.turningMotorInverted);

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
    }

    /**
     * Get the absolute encoder turn position
     * @return
     */
    public Angle getAbsoluteEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition().getValue();
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
