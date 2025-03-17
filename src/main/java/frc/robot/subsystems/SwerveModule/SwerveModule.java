package frc.robot.subsystems.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase implements SwerveModuleIO {
    private final SwerveModuleIO io;

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
    }

    public void resetDriveEncoder() {
        io.resetDriveEncoder();
    }

    public void resetTurningEncoder() {
        io.resetTurningEncoder();
    }

    public void resetEncoders() {
        io.resetEncoders();
    }

    public Angle getAngle() {
        return io.getAngle();
    }

    public void setTargetState(SwerveModuleState state, boolean force) {
        io.setTargetState(state, force);
    }

    public void setTargetState(SwerveModuleState state) {
        io.setTargetState(state);
    }

    public void stop() {
        io.stop();
    }

    public SwerveModuleState getTargetState() {
        return io.getTargetState();
    }

    public SwerveModuleState getRealState() {
        return io.getRealState();
    }

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        io.periodic();
    }
}
