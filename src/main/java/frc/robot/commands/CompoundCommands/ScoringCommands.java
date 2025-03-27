package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.CoralIntakeRollers;
import frc.robot.subsystems.CoralIntakeWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntakeArm.ArmPosition;
import lib.BlueShift.commands.RunForCommand;

public class ScoringCommands {
    public static Command scorePositionCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        // TODO: Leyva dice que asi si jala, depracaria scorePositionAutoCommand y se usaria este para los 2, falta probarlo
        return Commands.parallel(
            elevator.setSetpointCommand(level.getElevatorPosition()),
            arm.setSetpointCommand(level.getArmPosition()),
            wrist.setSetpointCommand(level.getWristPosition())
        );
    }

    public static Command scorePositionAutoCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> arm.setSetpoint(level.getArmPosition())),
            new InstantCommand(() -> elevator.setSetpoint(level.getElevatorPosition())),
            new InstantCommand(() -> wrist.setSetpointCommand(level.getWristPosition()))
        );
    }
    
    public static final Command scoreCommand(RobotState level, CoralIntakeArm arm, CoralIntakeRollers coralRollers) {
        if (level == RobotState.L1) {
            return new SequentialCommandGroup(
                new InstantCommand(coralRollers::setOut),
                new WaitCommand(0.25),
                new InstantCommand(coralRollers::stop)
            );
        } else {
            return new SequentialCommandGroup(
                new InstantCommand(coralRollers::setOut),
                new WaitCommand(0.075),
                new InstantCommand(()->arm.setSetpoint(ArmPosition.HORIZONTAL))
            );
        }
    }
}
