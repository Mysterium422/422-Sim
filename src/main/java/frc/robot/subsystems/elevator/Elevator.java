package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
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
        L1(35, 22),
        L2(25, 24),
        L3(25, 41),
        L4(60, 72),
        REEF_INTAKE(80, 72);

        public final double manipulatorAngle;
        public final double elevatorDistance;

        State(double manipulatorAngle, double elevatorDistance) {
            this.manipulatorAngle = manipulatorAngle;
            this.elevatorDistance = elevatorDistance;
        }
    }

    private @Getter @Setter State currentGoal = State.STOW;

    @Getter
    private boolean holdingCoral = false;

    private @Getter boolean holdingAlgae = false;

    private int coralFromFunnelAnimationTimer = 0;
    private int algaeAnimationTimer = 0;
    private Pose3d oldReefPosition = null;

    public Elevator(SwerveDriveSimulation driveSimulation) {
        this.driveSimulation = driveSimulation;
    }

    @Override
    public void periodic() {
        desiredManipulatorPosition = currentGoal.manipulatorAngle;
        desiredElevatorPosition = currentGoal.elevatorDistance;

        if (currentGoal.equals(State.REEF_INTAKE) && holdingCoral) {
            shootCoral();
        }

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

        if (coralFromFunnelAnimationTimer > 0) coralFromFunnelAnimationTimer--;

        if (algaeAnimationTimer > 0) algaeAnimationTimer--;
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
                new Translation3d(Inches.zero(), Inches.zero(), Inches.of(Math.max(0, currentElevatorPosition - 18))),
                new Rotation3d());
    }

    // x

    public Pose3d getStage2Pose() {
        return new Pose3d(
                new Translation3d(Inches.zero(), Inches.zero(), Inches.of(Math.max(0, currentElevatorPosition - 45))),
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
        coralFromFunnelAnimationTimer = 25;
    }

    public Pose3d getCoralPose() {
        Pose3d drivePose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());
        Pose3d manipulatorPose = drivePose.transformBy(new Transform3d(
                        new Translation3d(
                                0.285,
                                0,
                                0.203 + Inches.of(currentElevatorPosition).in(Meters)),
                        new Rotation3d(Degrees.zero(), Degrees.of(currentManipulatorPosition + 10), Degrees.of(0)))
                .plus(new Transform3d(new Translation3d(0.02, 0, 0.1), new Rotation3d())));

        if (coralFromFunnelAnimationTimer > 0) {
            Pose3d funnelPose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose())
                    .plus(new Transform3d(
                            new Translation3d(-0.08, 0, 0.33),
                            new Rotation3d(Degrees.zero(), Degrees.of(45), Degrees.of(0))));

            manipulatorPose = manipulatorPose.interpolate(funnelPose, coralFromFunnelAnimationTimer / 25.0);
        }

        // Pose3d rotatedCoralPose = manipulatorPose.rotateBy(new Rotation3d(
        //     Degrees.zero(), Degrees.of(30), Degrees.of(0)
        // ));
        return manipulatorPose;
    }

    public Pose3d getAlgaePose() {
        Pose3d drivePose = new Pose3d(driveSimulation.getSimulatedDriveTrainPose());

        Pose3d manipulatorPose = drivePose.transformBy(new Transform3d(
                new Translation3d(
                        0.35, 0, 0.42 + Inches.of(currentElevatorPosition).in(Meters)),
                new Rotation3d()));

        if (oldReefPosition != null && algaeAnimationTimer > 0) {
            manipulatorPose = manipulatorPose.interpolate(oldReefPosition, algaeAnimationTimer / 20.0);
        }

        // drivePose.transformBy(new Transform3d(
        //                 new Translation3d(
        //                         0.285,
        //                         0,
        //                         0.203 + Inches.of(currentElevatorPosition).in(Meters)),
        //                 new Rotation3d(Degrees.zero(), Degrees.of(currentManipulatorPosition + 10), Degrees.of(0)))
        //         .plus(new Transform3d(new Translation3d(0.02, 0, 0.1), new Rotation3d())));

        return manipulatorPose;
    }

    public void shoot() {
        shootCoral();
        shootAlgae();
    }

    public void shootCoral() {
        if (!holdingCoral) return;

        Pose3d coralPose = getCoralPose();
        Transform3d coralTransform = coralPose.minus(new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
        Translation3d velocity = new Translation3d(1.75, coralPose.getRotation());
        // SimulatedArena.getInstance().addGamePieceProjectile(MapleUtil.generateCoralAtPoint(getCoralPose(),
        // velocity));

        holdingCoral = false;

        if (currentGoal.equals(State.L4) && currentElevatorPosition == State.L4.elevatorDistance) {
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                            new Translation2d(0.46, 0),
                            // Obtain robot speed from drive simulation
                            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            Meters.of(2.1),
                            // The initial speed of the coral
                            MetersPerSecond.of(1),
                            // The coral is ejected vertically downwards
                            Degrees.of(-90)));
        } else {
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

    public void shootAlgae() {
        if (!holdingAlgae) return;

        Pose3d algaePose = getAlgaePose();
        Transform3d algaeTransform = algaePose.minus(new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
        // SimulatedArena.getInstance().addGamePieceProjectile(MapleUtil.generateCoralAtPoint(getCoralPose(),
        // velocity));

        holdingAlgae = false;

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                        // Obtain robot position from drive simulation
                        driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                        new Translation2d(algaeTransform.getX(), 0),
                        // Obtain robot speed from drive simulation
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        // Obtain robot facing from drive simulation
                        driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        // The height at which the coral is ejected
                        algaeTransform.getMeasureZ(),
                        // The initial speed of the coral
                        MetersPerSecond.of(1.4),
                        // The coral is ejected vertically downwards
                        Degrees.of(60)));
    }

    public boolean canHoldAlgae() {
        return currentGoal.equals(State.REEF_INTAKE) && !holdingCoral && !holdingAlgae;
    }

    public void getAlgaeFromReef(Pose3d reefPosition) {
        holdingAlgae = true;
        algaeAnimationTimer = 20;
        oldReefPosition = reefPosition;
    }
}
