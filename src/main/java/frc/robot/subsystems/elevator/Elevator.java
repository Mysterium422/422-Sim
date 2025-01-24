package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MapleUtil;
import lombok.Getter;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final SwerveDriveSimulation driveSimulation;

    @Getter
    private double currentElevatorPosition = 0;

    private double desiredElevatorPosition = 0;

    private double currentManipulatorPosition = 0;
    private double desiredManipulatorPosition = 0;

    public enum State {
        STOW(0, 0),
        L1(35, 24),
        L2(25, 40),
        L3(25, 41),
        L4(65, 71),
        REEF_INTAKE(80, 48);

        public final double manipulatorAngle;
        public final double elevatorDistance;

        State(double manipulatorAngle, double elevatorDistance) {
            this.manipulatorAngle = manipulatorAngle;
            this.elevatorDistance = elevatorDistance;
        }
    }

    private State currentGoal = State.STOW;
    @Getter private boolean holdingCoral = false;

    public Elevator(SwerveDriveSimulation driveSimulation) {
        this.driveSimulation = driveSimulation;
    }

    @Override
    public void periodic() {
        desiredManipulatorPosition = currentGoal.manipulatorAngle + 15;
        desiredElevatorPosition = currentGoal.elevatorDistance + 5;

        if (desiredManipulatorPosition != currentManipulatorPosition) {
            double difference = desiredManipulatorPosition - currentManipulatorPosition;
            double delta = Math.min(Math.abs(difference), 3.5);
            currentManipulatorPosition += Math.signum(difference) * delta;
        }

        if (desiredElevatorPosition != currentElevatorPosition) {
            double difference = desiredElevatorPosition - currentElevatorPosition;
            double delta = Math.min(Math.abs(difference), Math.max(0.9, Math.abs(difference * 0.04)));
            currentElevatorPosition += Math.signum(difference) * delta;
        }

        Logger.recordOutput("Elevator/goal", currentGoal);
        Logger.recordOutput("Elevator/current", currentElevatorPosition);
        Logger.recordOutput("Elevator/desired", desiredElevatorPosition);

        Logger.recordOutput("Manipulator/current", currentManipulatorPosition);
        Logger.recordOutput("Manipulator/desired", desiredManipulatorPosition);
    }

    // Max of stage 1: 26.687
    // Max of stage 2: 26.687
    // Max of stage 3: 18.25

    public Pose3d getCarriagePose() {
        return new Pose3d(
                new Translation3d(Inches.zero(), Inches.zero(), Inches.of(currentElevatorPosition)), new Rotation3d());
    }

    public Pose3d getStage3Pose() {
        return new Pose3d(
                new Translation3d(
                        Inches.zero(), Inches.zero(), Inches.of(Math.max(0, currentElevatorPosition - 18.25))),
                new Rotation3d());
    }

    public Pose3d getStage2Pose() {
        return new Pose3d(
                new Translation3d(
                        Inches.zero(), Inches.zero(), Inches.of(Math.max(0, currentElevatorPosition - 18.25 - 26.687))),
                new Rotation3d());
    }

    public Pose3d getManipulatorPose() {
        return new Pose3d( // Manipulator
                new Translation3d(
                        0.285, 0, 0.203 + Inches.of(currentElevatorPosition).in(Meters)),
                // new Translation3d(-0.31, 0, 0.19),
                new Rotation3d(
                        Units.Degrees.of(0), Units.Degrees.of(-90 + currentManipulatorPosition), Units.Degrees.of(0)));
    }

    public void toggle() {
        switch (currentGoal) {
            case STOW:
                currentGoal = State.L4;
                break;
            case L4:
                currentGoal = State.L3;
                break;
            case L3:
                currentGoal = State.L2;
                break;
            case L2:
                currentGoal = State.L1;
            case L1:
                currentGoal = State.REEF_INTAKE;
                break;
            case REEF_INTAKE:
                currentGoal = State.STOW;
        }
    }

    public void getCoralFromFunnel() {
        holdingCoral = true;
    }

    public Pose3d getCoralPose() {
        Pose3d drivePose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
        Pose3d manipulatorPose = drivePose.transformBy(new Transform3d(
            new Translation3d(0.285, 0, 0.203 + Inches.of(currentElevatorPosition).in(Meters)),
            new Rotation3d(Degrees.zero(), Degrees.of(currentManipulatorPosition + 10), Degrees.of(0))
        ).plus(new Transform3d(
            new Translation3d(0.02, 0, 0.1),
            new Rotation3d()
        )));
        // Pose3d rotatedCoralPose = manipulatorPose.rotateBy(new Rotation3d(
        //     Degrees.zero(), Degrees.of(30), Degrees.of(0)
        // ));
        return manipulatorPose;
    }

    public void shoot() {
        if (!holdingCoral) return;

        Pose3d coralPose = getCoralPose();
        Transform3d coralTransform = coralPose.minus(new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
        Translation3d velocity = new Translation3d(1.75, coralPose.getRotation());
        // SimulatedArena.getInstance().addGamePieceProjectile(MapleUtil.generateCoralAtPoint(getCoralPose(), velocity));

        SimulatedArena.getInstance()
    .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(coralTransform.getX(), 0),
        // Obtain robot speed from drive simulation
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        // Obtain robot facing from drive simulation
        driveSimulation.getSimulatedDriveTrainPose().getRotation(),
        // The height at which the coral is ejected
        coralTransform.getMeasureZ(),
        // The initial speed of the coral
        MetersPerSecond.of(1.8),
        // The coral is ejected vertically downwards
        coralTransform.getRotation().getMeasureAngle().unaryMinus()));
    }
}
