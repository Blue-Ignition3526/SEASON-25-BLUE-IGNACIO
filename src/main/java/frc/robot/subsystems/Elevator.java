package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Check unused stuff
public class Elevator extends SubsystemBase {
	// * Setpoints
	public static enum ElevatorPosition {
		L1(1.0),
		L2(2.0),
		L3(3.0),
		L4(4.5),
		HOME(0.0),
		SOURCE(1.0);

		private double position;

		private ElevatorPosition(double position) {
			this.position = position;
		}

		public double getPosition() {
			return position;
		}
	}

	// * Left motor
	private final TalonFX leftElevatorMotor;
	private final TalonFXConfiguration leftElevatorMotorConfig;

	// * Right motor
	private final TalonFX rightElevatorMotor;
	private final TalonFXConfiguration rightElevatorMotorConfig;

	// * Alerts
	private Alert alert_leftMotorUnreachable = new Alert(getName() + " left motor unreachable.", AlertType.kError);
	private Alert alert_rightMotorUnreachable = new Alert(getName() + " right motor unreachable.", AlertType.kError);

	// * Device check
	private Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

	// * MotionMagic controller
	private MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

	// * Status
	private ElevatorPosition m_setpoint = ElevatorPosition.HOME;

	public Elevator() {
    // * Right motor (MASTER)
    this.rightElevatorMotor = new TalonFX(ElevatorConstants.kRightMotorID, "*");

    // Configure motor
    this.rightElevatorMotorConfig = new TalonFXConfiguration();
    this.rightElevatorMotorConfig
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              .withSupplyCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
              .withSupplyCurrentLowerLimit(ElevatorConstants.kElevatorMotorLowerCurrentLimit)
              .withSupplyCurrentLowerTime(0.5)
      )
      .withClosedLoopRamps(
          new ClosedLoopRampsConfigs()
              .withVoltageClosedLoopRampPeriod(ElevatorConstants.kElevatorMotorRampRate)
      )
      .withVoltage(
          new VoltageConfigs()
              .withPeakForwardVoltage(12)
			  .withPeakForwardVoltage(-12)
      )
      .withMotorOutput(
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
			  .withInverted(InvertedValue.CounterClockwise_Positive)
      )
	  .withSlot0(
		new Slot0Configs()
			.withGravityType(GravityTypeValue.Elevator_Static)
			.withKP(ElevatorConstants.kMotionMagicKP)
			.withKV(ElevatorConstants.kMotionMagicKV)
			.withKG(ElevatorConstants.kMotionMagicKG)
	  )
	  .withFeedback(
		new FeedbackConfigs()
			.withSensorToMechanismRatio(ElevatorConstants.kSensorToMechanism)
	  )
	  .withMotionMagic(
		new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(ElevatorConstants.kMotionMagicVelocity)
			.withMotionMagicAcceleration(ElevatorConstants.kMotionMagicAcceleration)
	);


    // Apply right motor configuration
    this.rightElevatorMotor.getConfigurator().apply(rightElevatorMotorConfig);

    // * Left motor (FOLLOWER)
    this.leftElevatorMotor = new TalonFX(ElevatorConstants.kLeftMotorID, "*");

    // Configure motor
    this.leftElevatorMotorConfig = new TalonFXConfiguration();
    this.leftElevatorMotorConfig
		.withCurrentLimits(
			new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
				.withSupplyCurrentLowerLimit(ElevatorConstants.kElevatorMotorLowerCurrentLimit)
				.withSupplyCurrentLowerTime(0.5)
		)
		.withVoltage(
			new VoltageConfigs()
				.withPeakForwardVoltage(12)
		)
		.withMotorOutput(
			new MotorOutputConfigs()
				.withNeutralMode(NeutralModeValue.Brake)
	);
	
	// Make left motor follow right motor
	this.leftElevatorMotor.setControl(new Follower(ElevatorConstants.kRightMotorID, true));
		
    // Apply left motor configuration
    this.leftElevatorMotor.getConfigurator().apply(leftElevatorMotorConfig);

    // * Encoder
	this.rightElevatorMotor.setPosition(0);

    // Start device check
	deviceCheckNotifier.setName(getName() + " Device Check");
    deviceCheckNotifier.startPeriodic(Constants.deviceCheckPeriod);
  }

	private void deviceCheck() {
		if (rightElevatorMotor.isConnected()) {
			alert_rightMotorUnreachable.set(false);
		} else {
			alert_rightMotorUnreachable.set(true);
			DriverStation.reportError(alert_rightMotorUnreachable.getText(), false);
		}

		if (leftElevatorMotor.isConnected()) {
			alert_leftMotorUnreachable.set(false);
		} else {
			alert_leftMotorUnreachable.set(true);
			DriverStation.reportError(alert_leftMotorUnreachable.getText(), false);
		}
	}

	/**
	 * Get the current position of the elevator
	 * 
	 * @return
	 */
	public double getPosition() {
		return this.rightElevatorMotor.getPosition().getValue().in(Rotations);
	}

	/**
	 * Set the setpoint of the elevator
	 * 
	 * @param setpoint
	 */
	public void setSetpoint(ElevatorPosition setpoint) {
		this.m_setpoint = setpoint;
		this.rightElevatorMotor.setControl(positionControl.withSlot(0).withPosition(setpoint.getPosition()));
	}

	/**
	 * Stop the elevator
	 */
	public void stop() {
		this.rightElevatorMotor.setVoltage(0);
	}

	public void setVoltage(double voltage) {
		rightElevatorMotor.setVoltage(voltage);
	}

	public Command setVoltageCommand(double voltage) {
		return runOnce(() -> setVoltage(voltage));
	}

	public Command stopCommand() {
		return runOnce(this::stop);
	}

	public Command setSetpointCommand(ElevatorPosition setpoint) {
		return run(() -> setSetpoint(setpoint)).until(
			() -> Math.abs(rightElevatorMotor.getPosition().getValueAsDouble() - setpoint.getPosition()) < ElevatorConstants.kElevatorTolerance
		);
	}

	public Command resetElevatorPositionCommand() {
		return runOnce(() -> rightElevatorMotor.setPosition(0));
	}

	@Override
	public void periodic() {
		// Telemetry
		SmartDashboard.putString("Elevator/MotionMagicInfo", positionControl.getControlInfo().toString());
		SmartDashboard.putNumber("Elevator/AppliedOutput", rightElevatorMotor.get());
		SmartDashboard.putNumber("Elevator/CurrentPosition", getPosition());
		SmartDashboard.putNumber("Elevator/SetpointPosition", m_setpoint.getPosition());
		SmartDashboard.putNumber("Elevator/MotorSetpoint", positionControl.getPositionMeasure().baseUnitMagnitude());
		SmartDashboard.putNumber("Elevator/SetpointVoltage", rightElevatorMotor.getMotorVoltage().getValueAsDouble());
		SmartDashboard.putNumber("Elevator/LeftCurrent", leftElevatorMotor.getStatorCurrent().getValue().in(Amps));
		SmartDashboard.putNumber("Elevator/RightCurrent", rightElevatorMotor.getStatorCurrent().getValue().in(Amps));
	}
}
