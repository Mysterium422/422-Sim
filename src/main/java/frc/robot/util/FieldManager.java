package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class FieldManager extends SubsystemBase {

    private final SwerveDriveSimulation driveSimulation;

    public FieldManager(SwerveDriveSimulation driveSimulation) {
        this.driveSimulation = driveSimulation;
    }

    private boolean atRightStation = false;
    private boolean atLeftStation = false;

    @Override
    public void periodic() {
        if (driveSimulation
                        .getSimulatedDriveTrainPose()
                        .getTranslation()
                        .getDistance(FieldConstants.CoralStation.rightCenterFace.getTranslation())
                < 0.5) {
            if (!atRightStation) {
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(MapleUtil.generateCoralAtPoint(
                                new Pose3d(
                                        new Translation3d(0.622, 0.456, 1.3),
                                        new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(45))),
                                new Translation3d(
                                        2.3, new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(45)))));
            }
            atRightStation = true;
        } else {
            atRightStation = false;
        }

        if (driveSimulation
                        .getSimulatedDriveTrainPose()
                        .getTranslation()
                        .getDistance(FieldConstants.CoralStation.leftCenterFace.getTranslation())
                < 0.5) {
            if (!atLeftStation) {
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(MapleUtil.generateCoralAtPoint(
                                new Pose3d(
                                        new Translation3d(0.622, 7.596, 1.3),
                                        new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(135))),
                                new Translation3d(
                                        2.3, new Rotation3d(Degrees.zero(), Degrees.of(35), Degrees.of(-45)))));
            }
            atLeftStation = true;
        } else {
            atLeftStation = false;
        }
    }
}
