package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

//TODO: bruh
public class Climber extends SubsystemBase {
  public static enum ServoPosition {
    LOCKED(120),
    UNLOCKED(55);

    private final int position;
    private ServoPosition(int position) {
      this.position = position;
    }

    public int getPosition() {
      return position;
    }
  }

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  private final Servo servo;

  private ServoPosition servoPosition = ServoPosition.LOCKED;

  private final CANcoder encoder;

  private final PIDController pid;
  
  public Climber() {
    // Climber motor
    this.motor = new SparkFlex(ClimberConstants.kClimberMotorId, MotorType.kBrushless);

    // Configure motor
    this.motorConfig = new SparkFlexConfig(); 
    this.motorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit)
      .voltageCompensation(12)
      .openLoopRampRate(ClimberConstants.kClimberRampRate);
    
    this.motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Ratchet servo
    this.servo = new Servo(ClimberConstants.kServoPort);

    // Encoder
    this.encoder = new CANcoder(56, "*");

    this.pid = Constants.ClimberConstants.kPID;

    SmartDashboard.putNumber(getName() + "ServoAngleSetpoint", 90);
    SmartDashboard.putData(getName() + "SendServoCtrl", this.sendServoCtrlDashboardCommand());

    SmartDashboard.putData("Climber/unlock", unlockCommand());
    SmartDashboard.putData("Climber/lock", lockCommand());
  }

  private Command sendServoCtrlDashboardCommand() {
    return new InstantCommand(() -> servo.setAngle(SmartDashboard.getNumber(getName() + "ServoAngleSetpoint", 90)));
  }

  public void setVoltage(double volts) {
    if(volts == 0) {this.motor.setVoltage(0); return;}

    //if(volts > 0 && getPosition().in(Degrees) > 154.3 && servoPosition == ServoPosition.UNLOCKED) {
      this.motor.setVoltage(volts);
    //} else if(volts < 0 && getPosition().in(Degrees) < 292.3) {
    //  this.motor.setVoltage(volts);
    //}
  }

  public static double kClimberInMaxAngle = 172;
  public static double kClimberOutMaxAngle = 287;

  public void lock() {
    setServoPos(ServoPosition.LOCKED);
  }

  public void unlock() {
    setServoPos(ServoPosition.UNLOCKED);
  }

  public Angle getPosition() {
    return encoder.getAbsolutePosition().refresh().getValue();
  }

  public Command lockCommand() {
    return runOnce(this::lock);
  }

  public Command unlockCommand() {
    return runOnce(this::unlock);
  }

  public Command setVoltCommand(double volts) {
    return runOnce(() -> setVoltage(volts));
  }

  public Command setServo(ServoPosition pos) {
    return runOnce(() -> setServoPos(pos));
  }

  public Command stopCommand() {
    return new SequentialCommandGroup(
      setVoltCommand(0),
      new WaitCommand(0.5),
      lockCommand()
    );
  }

  public void setServoPos(ServoPosition pos) {
    servoPosition = pos;
    servo.setAngle(pos.getPosition());
  }

  @Override
  public void periodic() {
    double deg = getPosition().in(Degrees);

    SmartDashboard.putNumber("Climber/Angle", deg);

    SmartDashboard.putBoolean("Climber/Ready", MathUtil.isNear(kClimberOutMaxAngle, deg, 3));
    SmartDashboard.putBoolean("Climber/Climb", MathUtil.isNear(kClimberInMaxAngle, deg, 3));
  }
}
