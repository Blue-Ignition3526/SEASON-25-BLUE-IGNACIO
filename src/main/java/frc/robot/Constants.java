package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
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
            public static final Measure<DistanceUnit> kTrackWidth = Inches.of(26);
            public static final Measure<DistanceUnit> kWheelBase = Inches.of(26);
    
            // Create a kinematics instance with the positions of the swerve modules
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));
        }

        //* Swerve modules configuration
        public static final class SwerveModuleConstants {
            // * Motor config
            // Ramp rates
            public static final double kDriveMotorRampRate = 0;
            public static final double kTurningMotorRampRate = 0;

            // Current limits
            public static final int kDriveMotorCurrentLimit = 40;
            public static final int kTurningMotorCurrentLimit = 30;

            //* PID
            public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(1.57);

            //* Swerve modules options
            public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
                .setDriveMotorID(2)
                .setTurningMotorID(3)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(4, "*"))
                .setName("Front Left");

            public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
                .setDriveMotorID(5)
                .setTurningMotorID(6)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(7, "*"))
                .setName("Front Right");

            public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
                .setDriveMotorID(8)
                .setTurningMotorID(9)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(10, "*"))
                .setName("Back Left");

            public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
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
}
