package frc.robot.commands.CompoundCommands;

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

    public static Command outtakeCommand(ClimbertakeRollers rollers) {
        return rollers.setOutCommand();
    }

    public static Command grabCommand(ClimbertakePivot pivot, ClimbertakeRollers rollers) {
        return 
            intakeCommand(pivot, rollers)
            .until(rollers::hasPiece)
            .andThen(storeCommand(pivot, rollers));
    }
}
