package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.constants.SwerveModuleOptions;

public class SwerveModuleKrakenX60 extends SubsystemBase {
  // * Swerve module options
  SwerveModuleOptions options;

  // * Motors
  private final TalonFX turnMotor;
  private final TalonFX driveMotor;

  // * Absolute encoder
  private final CANcoder absoluteEncoder;

  // * Target state
  private SwerveModuleState targetState = new SwerveModuleState();

  // * PID Slot for turning
  private int turnPIDSlot = 0;

  public SwerveModuleKrakenX60(SwerveModuleOptions options) {
    // Store the options
    this.options = options;

    // Create the turn motor and configure it
    this.turnMotor = new TalonFX(options.turningMotorID, "*");
    // * new ClosedLoopGeneralConfigs().withContinuousWrap(true);
    new ClosedLoopGeneralConfigs().withContinuousWrap(true);
    // TODO: Apply brake to both motors
    // TODO: Apply PID Wrapping
    // TODO: Apply Slot0Configs for PID (try to implement MotionMagic too)
    // TODO: Apply remote cancoder feedback sensor
    // TODO: Apply correct configs

    // Create the drive motor and configure it
    this.driveMotor = new TalonFX(options.driveMotorID, "*");
    // TODO: Apply correct configs

    // Create the absolute encoder and configure it
    this.absoluteEncoder = new CANcoder(options.absoluteEncoderDevice.getDeviceID(), options.absoluteEncoderDevice.getCanbus());
    // TODO: Apply offsets
    // TODO: Apply correct configs
  }

  public Angle getAngle() {
    return turnMotor.getPosition().getValue();
  }
  
  public void setTargetState(SwerveModuleState state, boolean force) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001 || force) {
      stop();
      return;
    }

    state.optimize(Rotation2d.fromRotations(getAngle().in(Rotations)));

    this.targetState = state;

    driveMotor.set(state.speedMetersPerSecond / Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
    turnMotor.setControl(new PositionVoltage(state.angle.getRotations()).withSlot(turnPIDSlot));
  }

  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }

  @Override
  public void periodic() {
    
  }
}
