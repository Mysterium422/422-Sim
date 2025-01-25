package frc.robot.commands.funnel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.funnel.Funnel;

public class SetHandoff extends InstantCommand {

    public SetHandoff(Funnel funnel, boolean running) {
        super(() -> {
            funnel.isRunning = running;
        });
    }
}
