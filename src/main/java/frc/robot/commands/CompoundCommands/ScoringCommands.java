package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.CoralIntakeArm;
import frc.robot.subsystems.CoralIntakeRollers;
import frc.robot.subsystems.CoralIntakeWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralIntakeArm.ArmPosition;

public class ScoringCommands {
    public static class StateMachine {
        private static StateMachine instance = null;
        private RobotState currentState = RobotState.HOME;

        public StateMachine() {}

        public static StateMachine getInstance() {
            if (instance == null) instance = new StateMachine();
            return instance;
        }

        public void setState(RobotState state) {
            currentState = state;
            SmartDashboard.putString("RobotState/StateLevel", state.toString());
        }

        public RobotState getState() {
            return currentState;
        }
    }

    public static Command scorePositionCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        return Commands.parallel(
            new InstantCommand(() -> StateMachine.getInstance().setState(level)),
            elevator.setSetpointCommand(level.getElevatorPosition()),
            arm.setSetpointCommand(level.getArmPosition()),
            wrist.setSetpointCommand(level.getWristPosition())
        );
    }

    public static Command scorePositionAutoCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        return new ParallelCommandGroup(
            new InstantCommand(() -> StateMachine.getInstance().setState(level)),
            new InstantCommand(() -> wrist.setSetpoint(level.getWristPosition())),
            new InstantCommand(() -> arm.setSetpoint(level.getArmPosition())),
            new InstantCommand(() -> elevator.setSetpoint(level.getElevatorPosition()))
        );
    }

    public static Command scorePositionAutoCommandWithWait(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist) {
        return Commands.parallel(
            new InstantCommand(() -> StateMachine.getInstance().setState(level)),
            new RunCommand(() -> wrist.setSetpoint(level.getWristPosition())).until(wrist::atSetpoint),
            new RunCommand(() -> arm.setSetpoint(level.getArmPosition())).until(arm::atSetpoint),
            new RunCommand(() -> elevator.setSetpoint(level.getElevatorPosition())).until(elevator::atSetpoint)
        );
    }
    
    public static final Command scoreCommand(RobotState level, Elevator elevator, CoralIntakeArm arm, CoralIntakeWrist wrist, CoralIntakeRollers coralRollers) {
        if (level == RobotState.L1) {
            return new SequentialCommandGroup(
                new InstantCommand(coralRollers::setOutFASTER),
                new WaitCommand(0.5),
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
/*
else if (level == RobotState.SOURCE || level == RobotState.HOME) {
    return new SequentialCommandGroup(
        scorePositionCommand(level, elevator, arm, wrist),

        new InstantCommand(coralRollers::setOut),
        new WaitCommand(0.075),
        new InstantCommand(()->arm.setSetpoint(ArmPosition.HORIZONTAL))
    );
}
*/