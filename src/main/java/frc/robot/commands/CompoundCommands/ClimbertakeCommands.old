package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimbertakeConstants;
import frc.robot.subsystems.AlgaeClimbertakePivot;
import frc.robot.subsystems.AlgaeClimbertakeRollers;

public class ClimbertakeCommands {
    public static Command intakeCommand(AlgaeClimbertakePivot pivot, AlgaeClimbertakeRollers rollers) {
        return new ParallelCommandGroup(
            rollers.setInCommand(),
            pivot.setSetpointCommand(ClimbertakeConstants.Pivot.kIntakeAngle)
        );
    }

    public static Command storeCommand(AlgaeClimbertakePivot pivot, AlgaeClimbertakeRollers rollers) {
        return new ParallelCommandGroup(
            rollers.stopCommand(),
            pivot.setSetpointCommand(ClimbertakeConstants.Pivot.kStoreAngle)
        );
    }

    public static Command outtakeCommand(AlgaeClimbertakeRollers rollers) {
        return rollers.setOutCommand();
    }

    public static Command grabCommand(AlgaeClimbertakePivot pivot, AlgaeClimbertakeRollers rollers) {
        return 
            intakeCommand(pivot, rollers)
            .until(rollers::hasPiece)
            .andThen(storeCommand(pivot, rollers));
    }
}
