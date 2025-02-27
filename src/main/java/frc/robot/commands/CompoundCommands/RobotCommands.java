package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.CoralIntakeWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntakeArm.ArmPosition;
import frc.robot.subsystems.CoralIntakeWrist.WristPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class RobotCommands {
    public static Command stowCommand(CoralIntakeWrist wrist, CoralIntakeArm arm, Elevator elevator) {
        return new SequentialCommandGroup(
            wrist.setSetpointCommand(WristPosition.PARALLEL),
            arm.setSetpointCommand(ArmPosition.HORIZONTAL),
            elevator.setSetpointCommand(ElevatorPosition.HOME)
        );
    }

    public static Command intakeCoralCommand(CoralIntakeWrist wrist, CoralIntakeArm arm, Elevator elevator) {
        return new ParallelCommandGroup(
            wrist.setSetpointCommand(WristPosition.PARALLEL),
            elevator.setSetpointCommand(ElevatorPosition.SOURCE),
            arm.setSetpointCommand(ArmPosition.HIGH)
        );
    }
}
