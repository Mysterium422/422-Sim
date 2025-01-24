package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class Funnel extends SubsystemBase {

    private final SwerveDriveSimulation driveSimulation;
    private final Elevator elevator;

    public Funnel(SwerveDriveSimulation driveTrainSimulation, Elevator elevator) {
        this.driveSimulation = driveTrainSimulation;
        this.elevator = elevator;
    }

    private boolean holdingCoral = false;
    public boolean isRunning = false;
    private int waitBeforeNext = 0;

    @Override
    public void periodic() {
        if (!holdingCoral && waitBeforeNext == 0) {
            Set<GamePieceProjectile> allPieces = SimulatedArena.getInstance().gamePieceLaunched();
            Translation3d funnelPose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose())
                    .plus(new Transform3d(new Translation3d(-0.18, 0, 0.55), new Rotation3d()))
                    .getTranslation();

            for (GamePieceProjectile piece : allPieces) {
                if (!piece.gamePieceType.equals("Coral")) {
                    continue;
                }

                if (piece.getPose3d().getTranslation().getDistance(funnelPose) > 0.3) continue;

                if (piece.getVelocity3dMPS()
                                .toTranslation2d()
                                .toVector()
                                .dot(new Translation2d(
                                                1,
                                                driveSimulation
                                                        .getSimulatedDriveTrainPose()
                                                        .getRotation())
                                        .toVector())
                        < 0) {
                    continue;
                }

                holdingCoral = true;
                SimulatedArena.getInstance().removeProjectile(piece);
                break;
            }
        }
        if (holdingCoral) {
            if (isRunning) {
                if (elevator.getCurrentElevatorPosition() < 0.05) {
                    this.holdingCoral = false;
                } else {
                    Pose2d drivetrainPose = driveSimulation.getSimulatedDriveTrainPose();

                    this.holdingCoral = false;
                    waitBeforeNext = 50;

                    SimulatedArena.getInstance()
                            .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                    drivetrainPose.getTranslation(),
                                    new Translation2d(0, 0),
                                    driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                    drivetrainPose.getRotation(),
                                    Meters.of(0.35),
                                    MetersPerSecond.of(1.8),
                                    Units.Degrees.zero()));
                }
            }
        }

        if (waitBeforeNext > 0) {
            waitBeforeNext -= 1;
        }
    }

    public boolean isHoldingCoral() {
        return holdingCoral;
    }
}
