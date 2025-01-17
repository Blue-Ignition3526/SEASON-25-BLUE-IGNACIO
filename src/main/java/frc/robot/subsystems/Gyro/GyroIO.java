package frc.robot.subsystems.Gyro;

// import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    // @AutoLog
    class GyroIOInputs {
        double pitch;
        double yaw;
        double roll;

        double pitchVelocity;
        double yawVelocity;
        double rollVelocity;

        double accelerationX;
        double accelerationY;
        double accelerationZ;

        double velocityX;
        double velocityY;
        double velocityZ;
    }

    double getPitch();
    double getYaw();
    double getRoll();

    double getPitchVelocity();
    double getYawVelocity();
    double getRollVelocity();

    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();

    double getVelocityX();
    double getVelocityY();
    double getVelocityZ();

    Rotation2d getHeading();

    void reset();

    default void updateInputs(GyroIOInputs inputs) {};

    default void periodic() {};
}
