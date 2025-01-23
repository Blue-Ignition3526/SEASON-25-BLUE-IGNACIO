package frc.robot;

// import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import lib.team3526.constants.CTRECANDevice;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.constants.SwerveModuleOptions;
import lib.team3526.utils.SwerveChassis;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

public class Constants {
    //* Logging options
    public static final class Logging {
        public static final boolean kDebug = true;
    }

    //* Swerve Drive
    public static final class SwerveDrive {
        //* Gyroscope (Pigeon 2.0)
        public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34, "*");

        public static final double kJoystickDeadband = 0.1;
        //* Physical model of the robot
        public static final class PhysicalModel {
            //* MAX DISPLACEMENT SPEED (and acceleration)
            public static final Measure<LinearVelocityUnit> kMaxSpeed = MetersPerSecond.of(4.6);
            public static final Measure<LinearAccelerationUnit> kMaxAcceleration = MetersPerSecond.per(Second).of(kMaxSpeed.in(MetersPerSecond));

            //* MAX ROTATIONAL SPEED (and acceleration)
            public static final Measure<AngularVelocityUnit> kMaxAngularSpeed = DegreesPerSecond.of(360);
            public static final Measure<AngularAccelerationUnit> kMaxAngularAcceleration = DegreesPerSecond.per(Second).of(kMaxAngularSpeed.in(DegreesPerSecond));

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
        public static final class SwerveModules {
            //* PID
            public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(1.57);

            //* Global offset
            public static final Measure<AngleUnit> kGlobalOffset = Degrees.of(0);

            //* Swerve modules options
            public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setTurningMotorInverted(true)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(11, "*"))
                .setDriveMotorID(22)
                .setTurningMotorID(21)
                .setName("Front Left");

            public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setTurningMotorInverted(true)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(12, "*"))
                .setDriveMotorID(24)
                .setTurningMotorID(23)
                .setName("Front Right");

            public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setTurningMotorInverted(true)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(13, "*"))
                .setDriveMotorID(26)
                .setTurningMotorID(25)
                .setName("Back Left");

            public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setTurningMotorInverted(true)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(14, "*"))
                .setDriveMotorID(28)
                .setTurningMotorID(27)
                .setName("Back Right");
        }

        //* AUTONOMOUS
        public static final class Autonomous {
            public static final PIDConstants kTranslatePIDConstants = new PIDConstants(5.0, 0.0, 0.0);
            public static final PIDConstants kRotatePIDConstants = new PIDConstants(5.0, 0.0, 0.0);
            public static final Measure<LinearVelocityUnit> kMaxSpeedMetersPerSecond = MetersPerSecond.of(1);
        }
    }
}
