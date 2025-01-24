package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private double currentElevatorPosition = 0;
    private double desiredElevatorPosition = 0;

    private double currentManipulatorPosition = 0;
    private double desiredManipulatorPosition = 0;

    public enum State {
        STOW(0, 0),
        L1(35, 24),
        L2(35, 40),
        L3(35, 56),
        L4(45, 70),
        REEF_INTAKE(90, 50);

        public final double manipulatorAngle;
        public final double elevatorDistance;

        State(double manipulatorAngle, double elevatorDistance) {
            this.manipulatorAngle = manipulatorAngle;
            this.elevatorDistance = elevatorDistance;
        }
    }

    private State currentGoal = State.STOW;

    @Override
    public void periodic() {
        desiredManipulatorPosition = currentGoal.manipulatorAngle;
        desiredElevatorPosition = currentGoal.elevatorDistance;

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
}
