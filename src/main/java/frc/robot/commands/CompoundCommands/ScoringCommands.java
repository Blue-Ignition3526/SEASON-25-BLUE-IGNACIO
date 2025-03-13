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

public class ScoringCommands {
    public static Command scorePositionCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        // TODO: Leyva dice que asi si jala, depracaria scorePositionAutoCommand y se usaria este para los 2, falta probarlo
        return Commands.sequence(
            elevator.setSetpointCommand(level.getElevatorPosition()),
            arm.setSetpointCommand(level.getArmPosition()),
            Commands.waitSeconds(0.5),
            wrist.setSetpointCommand(level.getWristPosition())
        );
    }

    public static Command scorePositionAutoCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> arm.setSetpoint(level.getArmPosition())),
            new InstantCommand(() -> elevator.setSetpoint(level.getElevatorPosition())),
            new WaitCommand(0.5),
            new InstantCommand(() -> wrist.setSetpoint(level.getWristPosition()))
        );
    }
    
    public static final Command scoreCommand(RobotState level, CoralIntakeArm arm, CoralIntakeRollers coralRollers) {
        if (level == RobotState.L1) {
            return new SequentialCommandGroup(
                coralRollers.setOutCommand(),
                new WaitCommand(0.25),
                arm.setSetpointCommand(ArmPosition.HIGH),
                coralRollers.stopCommand()
            );
        } else {
            return new SequentialCommandGroup(
                coralRollers.setOutCommand(),
                new WaitCommand(0.5),
                arm.setSetpointCommand(ArmPosition.HORIZONTAL)
                // new WaitCommand(0.25),
                // coralRollers.stopCommand()
            );
        }
    }
}
