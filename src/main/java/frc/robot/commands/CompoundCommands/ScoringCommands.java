package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.CoralIntakeRollers;
import frc.robot.subsystems.CoralIntakeWrist;
import frc.robot.subsystems.Elevator;

public class ScoringCommands {
    public static Command scorePositionCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        return new SequentialCommandGroup(
            elevator.setSetpointCommand(level.getElevatorPosition()),
            arm.setSetpointCommand(level.getArmPosition()),
            new WaitCommand(0.5),
            // * If it is for trough, make the wrist parallel
            wrist.setSetpointCommand(level.getWristPosition())
        );
    }
    
    public static final Command scoreCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist, CoralIntakeRollers coralRollers) {
        if (level == RobotState.L1) {
            return new SequentialCommandGroup(
                coralRollers.setOutCommand(),
                new WaitCommand(0.5),
                RobotCommands.stowCommand(wrist, arm, elevator)
            );
        } else {
            return new SequentialCommandGroup(
                
            );
        }
    }
}
