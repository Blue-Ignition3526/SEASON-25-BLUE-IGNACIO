package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimbertakeConstants;
import frc.robot.subsystems.ClimbertakePivot;
import frc.robot.subsystems.ClimbertakeRollers;

public class ClimbertakeCommands {
    public static Command intakeCommand(ClimbertakePivot pivot, ClimbertakeRollers rollers) {
        return new ParallelCommandGroup(
            rollers.setInCommand(),
            pivot.setSetpointCommand(ClimbertakeConstants.Pivot.kOutAngle)
        );
    }

    public static Command storeCommand(ClimbertakePivot pivot, ClimbertakeRollers rollers) {
        return new ParallelCommandGroup(
            rollers.stopCommand(),
            pivot.setSetpointCommand(ClimbertakeConstants.Pivot.kInAngle)
        );
    }
}
