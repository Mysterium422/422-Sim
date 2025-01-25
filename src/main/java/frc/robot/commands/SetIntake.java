package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

public class SetIntake extends InstantCommand {

    public SetIntake(Intake intake) {
        super(() -> {
            intake.toggleIntakePosition();
        });
    }
}
