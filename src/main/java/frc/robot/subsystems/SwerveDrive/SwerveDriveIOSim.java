package frc.robot.subsystems.SwerveDrive;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveDriveIOSim implements SwerveDriveIO {
    private final SwerveDriveSimulation simulation;
    private final DriveTrainSimulationConfig config;

    public SwerveDriveIOSim() {
        this.config = DriveTrainSimulationConfig.Default()
            .withBumperSize(SwerveDriveConstants.PhysicalModel.kLengthWithBumpers, SwerveDriveConstants.PhysicalModel.kWidthWithBumpers)
            .withTrackLengthTrackWidth(SwerveDriveConstants.PhysicalModel.kWheelBase, SwerveDriveConstants.PhysicalModel.kTrackWidth)
            .withRobotMass(SwerveDriveConstants.PhysicalModel.kRobotMass)
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(COTS.ofMark4i(
                DCMotor.getKrakenX60(1),
                DCMotor.getNEO(1),
                COTS.WHEELS.COLSONS.cof, 2
            ));
        
        this.simulation = new SwerveDriveSimulation(config, new Pose2d());

        SimulatedArena.getInstance().addDriveTrainSimulation(simulation);
    }
}
