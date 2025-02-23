package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import lib.BlueShift.constants.CTRECANDevice;
import lib.BlueShift.constants.PIDFConstants;
import lib.BlueShift.constants.SwerveModuleOptions;
import lib.BlueShift.utils.SwerveChassis;
import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.config.PIDConstants;

public class Constants {
    //* Logging options
    public static final class Logging {
        public static final boolean kDebug = true;
        public static final boolean kUseURCL = true;
    }

    public static final class Vision {
        public static final class Limelight3G {
          public static final String kName = "limelight-threeg";
          public static final int kOdometryPipeline = 0;
          public static final int kSpeakerPipeline = 1;
          public static final int kViewfinderPipeline = 2;
        }
    }

    // * Climbertake
    public static final class ClimbertakeConstants {
        public static final class Pivot {
            // Motor
            public static final int kPivotMotorID = 50;
            public static final int kPivotMotorCurrentLimit = 40;
            public static final double kPivotMotorRampRate = 0.15;

            // Encoder
            public static final int kPivotEncoderPort = 0;
            public static final Angle kPivotEncoderOffset = Degrees.of(0);
            // public static final Angle kPivotUpperLimit = Degrees.of(180);
            // public static final Angle kPivotLowerLimit = Degrees.of(90);

            // PID Controller
            public static final ProfiledPIDController kPivotPIDController = new ProfiledPIDController(
                0.1, 0, 0, 
                new TrapezoidProfile.Constraints(10, 20)
            );
            // Feedforward
            // Calculated with https://www.reca.lc/arm?armMass=%7B%22s%22%3A8%2C%22u%22%3A%22kg%22%7D&comLength=%7B%22s%22%3A12%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A180%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A130%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
            // TODO: kS needs manual tuning
            public static final ArmFeedforward kPivotFeedforward = new ArmFeedforward(0.0, 0.67, 2.53);
            
            // Angles
            public static final Angle epsilon = Degrees.of(0);
            public static final Angle kOutAngle = Degrees.of(90 - 15);
            public static final Angle kInAngle = Degrees.of(90 + 5);
        }

        public static final class Rollers {
            // Motor and current limit
            public static final int kRollersMotorID = 31;
            public static final int kRollersMotorCurrentLimit = 30;
            public static final double kRollersMotorRampRate = 0;

            // Voltages and store current
            public static final double kRollersInVoltage = 6;
            public static final double kRollersOutVoltage = -kRollersInVoltage;
            public static final double kRollersStoreCurrent = 5;

            // Canandcolor piece sensor
            public static final int kPieceSensorID = 15;
            public static final double kPieceSensorLedBrightness = 0.5;
            public static final Color kAlgaeColor = new Color(0, 0.5, 0.5);
            public static final double kAlgaeColorThreshold = 0.15;
            public static final double kAlgaeProximityThreshold = 0.75;
        }
    }

    public static final class WristConstants {
        // Motor
        public static final int kWristMotorID = 33;
        public static final int kWristMotorCurrentLimit = 15;
        public static final double kWristMotorRampRate = 0.15;

        // Encoder
        public static final int kWristEncoderPort = 1;
        public static final Angle kWristEncoderOffset = Degrees.of(0);

        // Setpoint angles
        public static final Angle kMinAngle = Degrees.of(0);
        public static final Angle kMaxAngle = Degrees.of(90);

        // PID Controller
        // TODO: Tune
        public static final Angle epsilon = Degrees.of(1);
        public static final ProfiledPIDController kWristPIDController = new ProfiledPIDController(
            0.05, 0, 0, 
            new TrapezoidProfile.Constraints(10, 20)
        );
    }

    public static final class ArmPivotConstants {
        // Motor
        public static final int kArmPivotMotorID = 40;
        public static final int kArmPivotMotorCurrentLimit = 30;
        public static final double kArmPivotMotorRampRate = 0.15;

        // Encoder
        public static final int kArmPivotGyroID = 2;
        public static final Angle kArmPivotGyroOffset = Degrees.of(0);

        // Angle limits
        public static final Angle kMinAngle = Degrees.of(0);
        public static final Angle kMaxAngle = Degrees.of(90);

        // PID Controller
        // TODO: Tune
        public static final Angle epsilon = Degrees.of(1);
        public static final ProfiledPIDController kArmPivotPIDController = new ProfiledPIDController(
            0.05, 0, 0,
            new TrapezoidProfile.Constraints(10, 20)
        );
        public static final ArmFeedforward kArmPivotFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    }

    //* Swerve Drive
    public static final class SwerveDriveConstants {
        public static final class PoseControllers {
            public static final ProfiledPIDController rotationPID = new ProfiledPIDController(25, 0, 0, new TrapezoidProfile.Constraints(400, 180));
            public static final ProfiledPIDController translationPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(4.5, 3.3526));

            public static final double epsilon = 0.08;
            public static final double rotEpsilon = 0.5;
        }

        //* Gyroscope (Pigeon 2.0)
        public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34, "*");

        public static final double kJoystickDeadband = 0.1;
        //* Physical model of the robot
        public static final class PhysicalModel {
            //* MAX DISPLACEMENT SPEED (and acceleration)
            public static final Measure<LinearVelocityUnit> kMaxSpeed = MetersPerSecond.of(4.6);
            public static final Measure<LinearAccelerationUnit> kMaxAcceleration = MetersPerSecond.per(Second).of(20);

            //* MAX ROTATIONAL SPEED (and acceleration)
            public static final Measure<AngularVelocityUnit> kMaxAngularSpeed = DegreesPerSecond.of(360);
            public static final Measure<AngularAccelerationUnit> kMaxAngularAcceleration = DegreesPerSecond.per(Second).of(Math.pow(360, 2));

            // Drive wheel diameter
            public static final Measure<DistanceUnit> kWheelDiameter = Inches.of(4);

            // Gear ratios
            public static final double kDriveMotorGearRatio = 1.0 / 6.75; // 6.12:1 Drive
            public static final double kTurningMotorGearRatio = 1.0 / (150/7); // 12.8:1 Steering

            // Conversion factors (Drive Motor)
            public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameter.in(Meters);
            public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

            // Conversion factors (Turning Motor)
            public static final double kTurningEncoder_Rotation = kTurningMotorGearRatio;
            public static final double kTurningEncoder_RPS = kTurningEncoder_Rotation / 60.0;

            // Robot Without bumpers measures
            public static final Measure<DistanceUnit> kTrackWidth = Inches.of(23.08);
            public static final Measure<DistanceUnit> kWheelBase = Inches.of(22.64);
    
            // Create a kinematics instance with the positions of the swerve modules
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));

            // Rotation lock PIDF Constants
            public static final PIDFConstants kHeadingControllerPIDConstants = new PIDFConstants(0.1, 0.0, 0.0);

            // Physical model constants
            public static final double kRobotMassKg = 46;
        }

        //* Swerve modules configuration
        public static final class SwerveModuleConstants {
            //* PID
            public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(1.57);

            //* Global offset
            public static final Measure<AngleUnit> kGlobalOffset = Degrees.of(0);

            //* Swerve modules options
            public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setTurningMotorInverted(true)
                .setDriveMotorID(2)
                .setTurningMotorID(3)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(4, "*"))
                .setName("Front Left");

            public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setTurningMotorInverted(true)
                .setDriveMotorID(5)
                .setTurningMotorID(6)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(7, "*"))
                .setName("Front Right");

            public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setTurningMotorInverted(true)
                .setDriveMotorID(8)
                .setTurningMotorID(9)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(10, "*"))
                .setName("Back Left");

            public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setTurningMotorInverted(true)
                .setDriveMotorID(11)
                .setTurningMotorID(12)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(13, "*"))
                .setName("Back Right");
        }

        //* AUTONOMOUS
        public static final class AutonomousConstants {
            public static final PIDConstants kTranslatePIDConstants = new PIDConstants(5.0, 0.0, 0.0);
            public static final PIDConstants kRotatePIDConstants = new PIDConstants(5.0, 0.0, 0.0);
            public static final Measure<LinearVelocityUnit> kMaxSpeedMetersPerSecond = MetersPerSecond.of(1);
        }
    }

    public static final class IntakeCoralConstants {
        // Motor IDs
        public static final int kUpperMotorId = 10;
        public static final int kLowerMotorId = 11;

        // Motor limits
        public static final int kMotorCurrentLimit = 40;
        public static final double kMotorRampRate = 0.15;

        // Sensor
        public static final int kSensorId = 12;
        public static final double kPieceSensorLedBrightness = 0.5;
        public static final Color kCoralColor = new Color(1, 1, 1);
        public static final double kCoralColorThreshold = 0.15;
        public static final double kCoralProximityThreshold = 0.75;

        // Parameters
        public static final double kRollersInVoltage = 6;
        public static final double kRollersOutVoltage = -kRollersInVoltage;

    }
}
