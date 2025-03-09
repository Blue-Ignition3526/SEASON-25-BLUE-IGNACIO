package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveDriveConstants.SwerveModuleConstants;
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
		L1(118),
		L2(178),
		L3(324),
		L4(600),
		HOME(0.0),
		SOURCE(128);

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

	// * Position status signal (right motor integrated encoder)
	private final StatusSignal<Angle> positionStatusSignal;

	// * Status
	private double setpoint;
	private ElevatorPosition setpointEnum = null;
	private boolean pidEnabled = false;

	// * Alerts
	private Alert alert_leftMotorUnreachable = new Alert(getName() + " left motor unreachable.", AlertType.kError);
	private Alert alert_rightMotorUnreachable = new Alert(getName() + " right motor unreachable.", AlertType.kError);
	private Alert alert_encoderUnreachable = new Alert(getName() + " encoder unreachable.", AlertType.kError);
	private Alert alert_pidDisabled = new Alert(getName() + " PID disabled.", AlertType.kInfo);

	// * Device check
	private Notifier deviceCheckNotifier = new Notifier(this::deviceCheck);

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
      .withOpenLoopRamps(
          new OpenLoopRampsConfigs()
              .withVoltageOpenLoopRampPeriod(ElevatorConstants.kElevatorMotorRampRate)
      )
      .withVoltage(
          new VoltageConfigs()
              .withPeakForwardVoltage(12)
      )
      .withMotorOutput(
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
			  .withInverted(InvertedValue.Clockwise_Positive)
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
		.withOpenLoopRamps(
			new OpenLoopRampsConfigs()
				.withVoltageOpenLoopRampPeriod(ElevatorConstants.kElevatorMotorRampRate)
		)
		.withVoltage(
			new VoltageConfigs()
				.withPeakForwardVoltage(12)
		)
		.withMotorOutput(
			new MotorOutputConfigs()
				.withNeutralMode(NeutralModeValue.Brake)
				.withInverted(InvertedValue.Clockwise_Positive)
		);
	
	// Make left motor follow right motor
	this.leftElevatorMotor.setControl(new Follower(ElevatorConstants.kRightMotorID, true));
		
    // Apply left motor configuration
    this.leftElevatorMotor.getConfigurator().apply(leftElevatorMotorConfig);

    // * Encoder
    this.positionStatusSignal = rightElevatorMotor.getPosition();

    // * Set setpoint to initial postiion
    this.setpoint = getPosition();

    // * Elevator PID for tuning
    SmartDashboard.putData("Elevator/PID", ElevatorConstants.kElevatorPIDController);

    // Start device check
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
		return this.positionStatusSignal.getValue().in(Rotations);
	}

	/**
	 * Set the setpoint of the elevator
	 * 
	 * @param setpoint
	 */
	public void setSetpoint(double setpoint) {
		resetPID();
		this.setpoint = setpoint;
		this.setpointEnum = null;
		this.pidEnabled = true;
	}

	/**
	 * Set the setpoint of the elevator
	 * 
	 * @param setpoint
	 */
	public void setSetpoint(ElevatorPosition setpoint) {
		resetPID();
		this.setpoint = setpoint.getPosition();
		this.setpointEnum = setpoint;
		this.pidEnabled = true;
	}

	/**
	 * Stop the elevator
	 */
	public void stop() {
		this.pidEnabled = false;
		this.rightElevatorMotor.setVoltage(0);
	}

	public void setVoltage(double voltage) {
		this.pidEnabled = false;
		rightElevatorMotor.setVoltage(voltage);
	}

	public Command setVoltageCommand(double voltage) {
		return runOnce(() -> setVoltage(voltage));
	}

	public Command stopCommand() {
		return runOnce(this::stop);
	}

	public Command setSetpointCommand(double setpoint) {
		return runOnce(() -> setSetpoint(setpoint));
	}

	public Command setSetpointCommand(ElevatorPosition setpoint) {
		return runOnce(() -> setSetpoint(setpoint));
	}

	public Command resetElevatorPositionCommand() {
		return runOnce(() -> rightElevatorMotor.setPosition(0));
	}

	public void resetPID() {
		ElevatorConstants.kElevatorPIDController.reset(getPosition());
	}

	public Command resetPIDCommand() {
		return runOnce(this::resetPID);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		double currentPosition = getPosition();

		// Calculate needed voltage
		double pidOutputVolts = ElevatorConstants.kElevatorPIDController.calculate(currentPosition, setpoint);
		double resultVolts = pidOutputVolts;

		// Set the voltage to the motor
		// ! CHECK APPLIED VOLTAGE IN THE DASHBOARD FIRST BEFORE POWERING THE MOTOR
		if (pidEnabled)
			rightElevatorMotor.setVoltage(resultVolts);

		// Telemetry
		SmartDashboard.putNumber("Elevator/AppliedOutput", rightElevatorMotor.get());
		SmartDashboard.putNumber("Elevator/CurrentPosition", currentPosition);
		SmartDashboard.putNumber("Elevator/SetpointPosition", setpoint);
		SmartDashboard.putNumber("Elevator/SetpointVoltage", resultVolts);
		SmartDashboard.putNumber("Elevator/LeftCurrent", leftElevatorMotor.getStatorCurrent().getValue().in(Amps));
		SmartDashboard.putNumber("Elevator/RightCurrent", rightElevatorMotor.getStatorCurrent().getValue().in(Amps));
	}
}
