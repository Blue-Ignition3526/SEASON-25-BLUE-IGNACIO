package frc.robot.subsystems.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

public interface SwerveModuleIO {
    public void resetDriveEncoder();
    public void resetTurningEncoder();
    public default void resetEncoders() {
        resetDriveEncoder();
        resetTurningEncoder();
    };

    public Angle getAngle();
    
    public void setTargetState(SwerveModuleState state, boolean force);
    public default void setTargetState(SwerveModuleState state) {
        setTargetState(state, false);
    };

    public void stop();

    public SwerveModuleState getTargetState();
    public SwerveModuleState getRealState();
    
    public SwerveModulePosition getPosition();

    public default void periodic() {};
}
