package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.elevator.Elevator;
import java.util.ArrayList;
import lombok.Getter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class FieldManager extends SubsystemBase {

    private final SwerveDriveSimulation driveSimulation;
    private final Elevator elevator;

    private boolean atRightStation = false;
    private boolean atLeftStation = false;

    @Getter
    private final ArrayList<Pose3d> reefAlgae;

    public static final Pose2d[] centerFaces = new Pose2d[6];

    public FieldManager(SwerveDriveSimulation driveSimulation, Elevator elevator) {
        centerFaces[0] =
                new Pose2d(Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180));
        centerFaces[1] =
                new Pose2d(Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120));
        centerFaces[2] =
                new Pose2d(Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60));
        centerFaces[3] =
                new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0));
        centerFaces[4] =
                new Pose2d(Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60));
        centerFaces[5] =
                new Pose2d(Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120));

        this.driveSimulation = driveSimulation;
        this.elevator = elevator;
        reefAlgae = new ArrayList<>();

        for (int i = 0; i < centerFaces.length; i++) {
            Pose3d algaePose = new Pose3d(centerFaces[i]);
            if (i % 2 == 0) {
                algaePose = algaePose.transformBy(new Transform3d(new Translation3d(-0.15, 0, 0.87), new Rotation3d()));
            } else {
                algaePose = algaePose.transformBy(new Transform3d(new Translation3d(-0.15, 0, 1.29), new Rotation3d()));
            }

            reefAlgae.add(algaePose);
        }
    }

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

        if (elevator.canHoldAlgae()) {
            for (int i = 0; i < reefAlgae.size(); i++) {
                Pose3d algae = reefAlgae.get(i);

                if (elevator.getCoralPose().getTranslation().getDistance(algae.getTranslation()) < 0.28) {
                    reefAlgae.remove(i);
                    elevator.getAlgaeFromReef(algae);
                    break;
                }
            }
        }
    }

    public void resetField() {
        reefAlgae.clear();

        for (int i = 0; i < centerFaces.length; i++) {
            Pose3d algaePose = new Pose3d(centerFaces[i]);
            if (i % 2 == 1) {
                algaePose = algaePose.transformBy(new Transform3d(new Translation3d(-0.15, 0, 0.87), new Rotation3d()));
            } else {
                algaePose = algaePose.transformBy(new Transform3d(new Translation3d(-0.15, 0, 1.29), new Rotation3d()));
            }

            reefAlgae.add(algaePose);
        }
    }
}
