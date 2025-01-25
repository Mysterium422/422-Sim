package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorPosition extends InstantCommand {

    public SetElevatorPosition(Elevator elevator, Elevator.State state) {
        super(() -> {
            elevator.setCurrentGoal(state);
        });
    }
}
