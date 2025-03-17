package frc.robot.subsystems.SwerveModule;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final SwerveModuleSimulation simulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turningMotor;

    private SwerveModuleState targetState = new SwerveModuleState();

    private final PIDController turningPID;

    public final String name;

    public SwerveModuleIOSim(SwerveModuleSimulation simulation, String name) {
        this.simulation = simulation;

        this.driveMotor = simulation.useGenericMotorControllerForDrive(); //.withCurrentLimit(Amps.of(SwerveDriveConstants.SwerveModuleConstants.kDriveMotorCurrentLimit));
        this.turningMotor = simulation.useGenericControllerForSteer(); //.withCurrentLimit(Amps.of(SwerveDriveConstants.SwerveModuleConstants.kTurningMotorCurrentLimit));

        this.turningPID = SwerveDriveConstants.SwerveModuleConstants.kTurningPIDConstants.toPIDController();

        this.name = name;
    }

    public void resetDriveEncoder() {
        driveMotor.updateControlSignal(
            Degrees.of(0),
            DegreesPerSecond.of(0), 
            Degrees.of(0),
            DegreesPerSecond.of(0)
        );
    }

    public void resetTurningEncoder() {
        turningMotor.updateControlSignal(
            Degrees.of(0),
            DegreesPerSecond.of(0), 
            Degrees.of(0),
            DegreesPerSecond.of(0)
        );
    }

    public Angle getAngle() {
        return simulation.getSteerAbsoluteAngle();
    }

    public void setTargetState(SwerveModuleState state, boolean force) {
        if (Math.abs(state.speedMetersPerSecond) < Double.MIN_VALUE && !force) {
            stop();
            return;
        }

        // Optimize the angle
        state.optimize(Rotation2d.fromRotations(getAngle().in(Rotations)));
        
        // Scale the target state for smoother movement
        state.cosineScale(Rotation2d.fromRotations(getAngle().in(Rotations)));

        // Set the target state for safekeeping
        this.targetState = state;

        // If it's locked, don't move
        // TODO: CHECK IF I SHOULD REMOVE THIS
        //if (lock.isLocked()) return;

        // Set motor speeds
        driveMotor.requestVoltage(Volts.of(state.speedMetersPerSecond / Constants.SwerveDriveConstants.PhysicalModel.kMaxSpeed.in(MetersPerSecond) * 12));
        turningMotor.requestVoltage(Volts.of(turningPID.calculate(getAngle().in(Rotations), state.angle.getRotations()) * 12));
    }

    public void stop() {
        driveMotor.requestVoltage(Volts.of(0));
        turningMotor.requestVoltage(Volts.of(0));
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModuleState getRealState() {
        return new SwerveModuleState(
            simulation.getDriveEncoderUnGearedSpeed().in(RPM) * Constants.SwerveDriveConstants.PhysicalModel.kDriveEncoder_RPMToMeterPerSecond,
            new Rotation2d(getAngle())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            simulation.getDriveEncoderUnGearedPosition().in(Rotations) * Constants.SwerveDriveConstants.PhysicalModel.kDriveEncoder_RotationToMeter,
            new Rotation2d(getAngle())
        );
    }

    @Override
    public void periodic() {
        Logger.recordOutput("SwerveDrive/" + name + "/RealState", this.getRealState());
        Logger.recordOutput("SwerveDrive/" + name + "/TargetState", this.getTargetState());
    }
}
