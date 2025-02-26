package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.CoralIntakeRollers;
import frc.robot.subsystems.CoralIntakeWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.CoralIntakeArm.ArmPosition;
import frc.robot.subsystems.CoralIntakeWrist.WristPosition;
import lib.BlueShift.control.SpeedAlterator;

public class ScoringCommands {
    public static Command scorePositionCommand(ReefLevel level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setSetpointCommand(level.getElevatorPosition()),
                arm.setSetpointCommand(level.getArmPosition())
            ),
            new WaitCommand(0.5),
            // * If it is for trough, make the wrist parallel
            wrist.setSetpointCommand(level == ReefLevel.L1 ? WristPosition.PARALLEL : WristPosition.PERPENDICULAR)
        );
    }
    
    public static final Command scoreCommand(ReefLevel level, SwerveDrive drive, SpeedAlterator backUpAlterator, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist, CoralIntakeRollers coralRollers) {
        if (level == ReefLevel.L1) {
            return new SequentialCommandGroup(
                coralRollers.setOutCommand(),
                new WaitCommand(0.5),
                RobotCommands.stowCommand(wrist, arm, elevator)
            );
        } else {
            return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    coralRollers.setOutCommand(),
                    arm.setSetpointCommand(ArmPosition.HORIZONTAL)  
                ),

                new WaitCommand(0.05),

                drive.enableSpeedAlteratorCommand(backUpAlterator),

                new WaitCommand(0.5),

                drive.disableSpeedAlteratorCommand(),

                RobotCommands.stowCommand(wrist, arm, elevator)
            );
        }
    }

    /*
     * Bind score sequence to 2 presses of the passed trigger
     */
    public static final void bindScoreSequence(Trigger trigger, ReefLevel reefLevel, SwerveDrive drive, SpeedAlterator backUpAlterator, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist, CoralIntakeRollers coralRollers) {
        trigger.onTrue(new SequentialCommandGroup(
            scorePositionCommand(reefLevel, elevator, arm, wrist),
            new WaitUntilCommand(trigger::getAsBoolean),
            scoreCommand(reefLevel, drive, backUpAlterator, elevator, arm, wrist, coralRollers)
        ));
    }
}
