package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorShoot extends InstantCommand {

    public ElevatorShoot(Elevator elevator) {
        super(() -> {
            elevator.shoot();
        });
    }
}
