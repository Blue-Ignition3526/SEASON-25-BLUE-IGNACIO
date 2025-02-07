package frc.robot.subsystems;

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
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
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
    private final SparkMax turningMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;

    //* PID Controller for turning
    public final SparkClosedLoopController turningPID;

    //* Absolute encoder
    private final CANcoder absoluteEncoder;

    //* Target state
    private SwerveModuleState targetState = new SwerveModuleState();

    /**
     * Create a new swerve module with the provided options
     * @param options
     */
    public SwerveModule(SwerveModuleOptions options) {
        // Motor configs
        this.turnConfig = new SparkMaxConfig();
        this.driveConfig = new SparkMaxConfig();

        // Store the options
        this.options = options;

        // Create the motors
        this.driveMotor = new SparkMax(options.driveMotorID, MotorType.kBrushless);
        this.turningMotor = new SparkMax(options.turningMotorID, MotorType.kBrushless);

        // Invert turn motor
        this.turnConfig.inverted(options.turningMotorInverted);

        // Get and configure the encoders
        this.driveConfig.encoder.positionConversionFactor(Constants.SwerveDriveConstants.PhysicalModel.kDriveEncoder_RotationToMeter); 
        this.driveConfig.encoder.velocityConversionFactor(Constants.SwerveDriveConstants.PhysicalModel.kDriveEncoder_RPMToMeterPerSecond);
        
        // this.driveConfig.encoder.inverted(true);
        this.driveConfig.inverted(options.driveMotorInverted);

        this.turnConfig.encoder.positionConversionFactor(Constants.SwerveDriveConstants.PhysicalModel.kTurningEncoder_Rotation); 
        this.turnConfig.encoder.velocityConversionFactor(Constants.SwerveDriveConstants.PhysicalModel.kTurningEncoder_RPS);

        // Get the PID controller for the turning motor
        this.turningPID = turningMotor.getClosedLoopController();

        // Configure the PID controller
        this.turnConfig.closedLoop.p(Constants.SwerveDriveConstants.SwerveModuleConstants.kTurningPIDConstants.kP);
        this.turnConfig.closedLoop.i(Constants.SwerveDriveConstants.SwerveModuleConstants.kTurningPIDConstants.kI);
        this.turnConfig.closedLoop.d(Constants.SwerveDriveConstants.SwerveModuleConstants.kTurningPIDConstants.kD);
        this.turnConfig.closedLoop.positionWrappingInputRange(0, 1);
        this.turnConfig.closedLoop.positionWrappingEnabled(true);

        // Configure the absolute encoder
        this.absoluteEncoder = new CANcoder(options.absoluteEncoderDevice.getDeviceID(), options.absoluteEncoderDevice.getCanbus());

        this.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Reset the encoders
        resetEncoders();
        
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
        this.turningMotor.getEncoder().setPosition(getAbsoluteEncoderPosition().in(Rotations));
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
        return Rotations.of(this.turningMotor.getEncoder().getPosition());
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
        if (Math.abs(state.speedMetersPerSecond) < 0.001 || force) {
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
        turningPID.setReference(state.angle.getRotations(), ControlType.kPosition);
    }

    /**
     * Stop the module (set the speed of the motors to 0)
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
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
