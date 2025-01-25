package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private double currentPosition = 0;
    private double desiredPosition = 0;

    private State currentGoal = State.STOW;

    private final SwerveDriveSimulation driveSimulation;
    private final IntakeSimulation intakeSimulation;

    public enum State {
        STOW,
        INTAKING
    }

    public Intake(SwerveDriveSimulation driveSimulation) {
        this.driveSimulation = driveSimulation;
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                "Algae",
                // Specify the drivetrain to which this intake is attached
                driveSimulation,
                // Width of the intake
                Meters.of(0.49),
                // The extension length of the intake beyond the robot's frame (when activated)
                Meters.of(0.21),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.BACK,
                // The intake can hold up to 1 note
                1);
        intakeSimulation.stopIntake();
    }

    Set<GamePieceOnFieldSimulation> previousPieces = new HashSet<>();

    private boolean holdingInPreviousCycle;
    private Pose3d algaeAnimationPose = new Pose3d();
    private int algaeAnimationTimer = 0;

    @Override
    public void periodic() {
        switch (currentGoal) {
            case STOW:
                desiredPosition = 0;
                break;
            case INTAKING:
                desiredPosition = 50;
                break;
        }

        if (desiredPosition != currentPosition) {
            double difference = desiredPosition - currentPosition;
            double delta = Math.min(Math.abs(difference), 5);
            currentPosition += Math.signum(difference) * delta;
        }

        Set<GamePieceOnFieldSimulation> currentPieces = new HashSet<>();

        for (GamePieceOnFieldSimulation gamePiece : SimulatedArena.getInstance().gamePiecesOnField()) {
            if (!gamePiece.type.equals("Algae")) {
                continue;
                // && !gamePiece.type.equals("CoralAlgaeStack")
            }

            currentPieces.add(gamePiece);
        }

        previousPieces.removeAll(currentPieces);

        Logger.recordOutput("Intake/PreviousSize", previousPieces.size());
        Logger.recordOutput("Intake/CurrentSize", currentPieces.size());

        if (!previousPieces.isEmpty() && !holdingInPreviousCycle && isHoldingAlgae()) {
            algaeAnimationPose = previousPieces.toArray(new GamePieceOnFieldSimulation[0])[0].getPose3d();
            algaeAnimationTimer = 15;
        }

        holdingInPreviousCycle = isHoldingAlgae();
        previousPieces = currentPieces;

        if (algaeAnimationTimer > 0) algaeAnimationTimer--;

        Logger.recordOutput("Intake/goal", currentGoal);
        Logger.recordOutput("Intake/current", currentPosition);
        Logger.recordOutput("Intake/desired", desiredPosition);
    }

    public void toggleIntakePosition() {
        if (currentGoal.equals(State.INTAKING)) {
            currentGoal = State.STOW;
            intakeSimulation.stopIntake();
            if (isHoldingAlgae()) {
                intakeSimulation.obtainGamePieceFromIntake();
                Pose2d drivetrainPose = driveSimulation.getSimulatedDriveTrainPose();
                // Translation2d target = FieldConstants.Processor.centerFace.getTranslation();
                // double distance = drivetrainPose.getTranslation().getDistance(target);

                // Translation2d toTarget = target.minus(drivetrainPose.getTranslation());
                // // Rotation that points from robot to target
                // Rotation2d angleToTarget = new Rotation2d(toTarget.getX(), toTarget.getY());
                // // Difference between robot's heading and the angle to target
                // Rotation2d headingDiff = angleToTarget.minus(drivetrainPose.getRotation());
                // double headingDiffDeg = headingDiff.getDegrees();

                // if (distance < 1 && Math.abs(headingDiffDeg) < 10) {
                //     SimulatedArena.getInstance().addGamePieceProjectile(
                //         new ReefscapeAlgaeOnFly(
                //             drivetrainPose.getTranslation(),
                //             new Translation2d(1.55, 0),
                //             driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                //             drivetrainPose.getRotation().plus(Rotation2d.fromDegrees(180)),
                //             Meters.of(0.275),
                //             MetersPerSecond.of(1),
                //             Units.Degrees.of(30)
                //         )
                //     );
                // } else {
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                                drivetrainPose.getTranslation(),
                                new Translation2d(0.65, 0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                drivetrainPose.getRotation().plus(Rotation2d.fromDegrees(180)),
                                Meters.of(0.275),
                                MetersPerSecond.of(1.2),
                                Units.Degrees.of(15)));
                // }
            }
        } else {
            currentGoal = State.INTAKING;
            intakeSimulation.startIntake();
        }
    }

    public Pose3d getPose() {
        return new Pose3d( // Algae Intake
                new Translation3d(-0.31, 0, 0.19),
                new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(-150 + currentPosition), Units.Degrees.of(180)));
    }

    public Pose3d getAlgaePosition() {
        Pose3d algaePose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose())
            .plus(new Transform3d(new Translation3d(-0.55, 0, 0.275), new Rotation3d()));

        if (algaeAnimationTimer > 0 && algaeAnimationPose != null) {
            algaePose = algaePose.interpolate(algaeAnimationPose, algaeAnimationTimer / 15.0);
        }
        return algaePose; 
    }

    public boolean isHoldingAlgae() {
        return intakeSimulation.getGamePiecesAmount() > 0;
    }
}
