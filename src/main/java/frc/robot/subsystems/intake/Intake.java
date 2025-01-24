package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private double currentPosition = 0;
    private double desiredPosition = 0;

    private State currentGoal = State.STOW;

    private final SwerveDriveSimulation driveTrainSimulation;
    private final IntakeSimulation intakeSimulation;

    public enum State {
        STOW,
        INTAKING
    }

    public Intake(SwerveDriveSimulation driveTrainSimulation) {
        this.driveTrainSimulation = driveTrainSimulation;
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                // Specify the type of game pieces that the intake can collect
                "Algae",
                // Specify the drivetrain to which this intake is attached
                driveTrainSimulation,
                // Width of the intake
                Meters.of(0.52),
                // The extension length of the intake beyond the robot's frame (when activated)
                Meters.of(0.23),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.BACK,
                // The intake can hold up to 1 note
                1);
        intakeSimulation.stopIntake();
    }

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
                Pose2d drivetrainPose = driveTrainSimulation.getSimulatedDriveTrainPose();
                Translation2d target = FieldConstants.Processor.centerFace.getTranslation();
                double distance = drivetrainPose.getTranslation().getDistance(target);

                Translation2d toTarget = target.minus(drivetrainPose.getTranslation());
                // Rotation that points from robot to target
                Rotation2d angleToTarget = new Rotation2d(toTarget.getX(), toTarget.getY());
                // Difference between robot's heading and the angle to target
                Rotation2d headingDiff = angleToTarget.minus(drivetrainPose.getRotation());
                double headingDiffDeg = headingDiff.getDegrees();

                // if (distance < 1 && Math.abs(headingDiffDeg) < 10) {
                //     SimulatedArena.getInstance().addGamePieceProjectile(
                //         new ReefscapeAlgaeOnFly(
                //             drivetrainPose.getTranslation(),
                //             new Translation2d(1.55, 0),
                //             driveTrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
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
                                driveTrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
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
                // new Translation3d(0, 0, 0),
                new Translation3d(-0.31, 0, 0.19),
                new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(-150 + currentPosition), Units.Degrees.of(180)));
    }

    public boolean isHoldingAlgae() {
        return intakeSimulation.getGamePiecesAmount() > 0;
    }
}
