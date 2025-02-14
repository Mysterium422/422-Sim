// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetIntake;
import frc.robot.commands.elevator.ElevatorShoot;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.funnel.SetHandoff;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.*;
import frc.robot.util.EricNubControls;
import frc.robot.util.FieldManager;
import frc.robot.util.LocalADStarAK;
import java.util.List;
import lombok.Getter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final @Getter Drive drive;
    private final Vision vision;

    private final Intake intake;

    @Getter
    private final Elevator elevator;

    private final @Getter Funnel funnel;
    private final FieldManager fieldManager;
    private final EricNubControls ericNubControls = new EricNubControls();

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandPS5Controller controller = new CommandPS5Controller(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

                break;
        }

        intake = new Intake(driveSimulation);
        elevator = new Elevator(driveSimulation);
        funnel = new Funnel(driveSimulation, elevator);
        fieldManager = new FieldManager(driveSimulation, elevator);

        configureNamedCommands();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureNamedCommands() {

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                driveSimulation::getSimulatedDriveTrainPose,
                (pose) -> {
                    driveSimulation.setSimulationWorldPose(pose);
                    drive.setPose(driveSimulation.getSimulatedDriveTrainPose());
                },
                drive::getChassisSpeeds,
                drive::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                Drive.PP_CONFIG,
                () -> false,
                drive);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        NamedCommands.registerCommand("L4", new SetElevatorPosition(elevator, Elevator.State.L4));
        NamedCommands.registerCommand("ElevatorStow", new SetElevatorPosition(elevator, Elevator.State.STOW));
        NamedCommands.registerCommand("ElevatorShoot", new ElevatorShoot(elevator));
        NamedCommands.registerCommand("FunnelStart", new SetHandoff(funnel, true));
        NamedCommands.registerCommand("FunnelStop", new SetHandoff(funnel, false));
        NamedCommands.registerCommand("IntakeToggle", new SetIntake(intake));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> {
                    double val = ericNubControls.addDeadzoneScaled(controller.getLeftY(), 0.1);
                    return Math.signum(val) * Math.pow(val, 2);
                },
                () -> {
                    double val = ericNubControls.addDeadzoneScaled(controller.getLeftX(), 0.1);
                    return Math.signum(val) * Math.pow(val, 2);
                },
                () -> {
                    double val = ericNubControls.addDeadzoneScaled(controller.getRightX(), 0.03);
                    return -Math.signum(val) * Math.pow(val, 4);
                }));

        controller
                .L1()
                .whileTrue(new InstantCommand(() -> {
                    funnel.isRunning = true;
                }))
                .onFalse(new InstantCommand(() -> {
                    funnel.isRunning = false;
                }));

        controller
                .povDown()
                .whileTrue(new InstantCommand(() -> {
                    elevator.setCurrentGoal(Elevator.State.L1);
                }))
                .onFalse(new InstantCommand(() -> {
                    if (elevator.getCurrentGoal().equals(Elevator.State.L1)) {
                        elevator.setCurrentGoal(Elevator.State.STOW);
                    }
                }));

        controller
                .povLeft()
                .whileTrue(new InstantCommand(() -> {
                    elevator.setCurrentGoal(Elevator.State.L2);
                }))
                .onFalse(new InstantCommand(() -> {
                    if (elevator.getCurrentGoal().equals(Elevator.State.L2)) {
                        elevator.setCurrentGoal(Elevator.State.STOW);
                    }
                }));

        controller
                .povRight()
                .whileTrue(new InstantCommand(() -> {
                    elevator.setCurrentGoal(Elevator.State.L3);
                }))
                .onFalse(new InstantCommand(() -> {
                    if (elevator.getCurrentGoal().equals(Elevator.State.L3)) {
                        elevator.setCurrentGoal(Elevator.State.STOW);
                    }
                }));

        controller
                .povUp()
                .whileTrue(new InstantCommand(() -> {
                    elevator.setCurrentGoal(Elevator.State.L4);
                }))
                .onFalse(new InstantCommand(() -> {
                    if (elevator.getCurrentGoal().equals(Elevator.State.L4)) {
                        elevator.setCurrentGoal(Elevator.State.STOW);
                    }
                }));

        controller
                .L2()
                .whileTrue(new InstantCommand(() -> {
                    elevator.setCurrentGoal(Elevator.State.REEF_INTAKE);
                }))
                .onFalse(new InstantCommand(() -> {
                    if (elevator.getCurrentGoal().equals(Elevator.State.REEF_INTAKE)) {
                        elevator.setCurrentGoal(Elevator.State.STOW);
                    }
                }));

        controller.circle().onTrue(new InstantCommand(() -> {
            //     elevator.toggle();

            //     DriverStation.reportWarning("A Button Pressed!!", true);
            //     SimulatedArena.getInstance()
            //             .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
            //                     new Translation2d(4, 4),
            //                     new Translation2d(),
            //                     new ChassisSpeeds(),
            //                     new Rotation2d(),
            //                     Units.Meters.of(2),dds
            //                     Units.MetersPerSecond.of(0),
            //                     Units.Degrees.of(0)));
        }));

        controller.R1().onTrue(new InstantCommand(() -> {
            elevator.shoot();
        }));

        controller.R2().onTrue(new InstantCommand(() -> {
            intake.toggleIntakePosition();
        }));
        // Lock to 0° when A button is held
        // controller
        //         .a()
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.setPose(driveSimulation.getSimulatedDriveTrainPose());
        SimulatedArena.getInstance().resetFieldForAuto();

        fieldManager.resetField();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());

        Logger.recordOutput("FieldSimulation/FinalComponentPoses", new Pose3d[] {
            intake.getPose(),
            elevator.getStage2Pose(), // Stage 2
            elevator.getStage3Pose(), // Stage 3
            elevator.getCarriagePose(), // Carriage
            elevator.getManipulatorPose(),
            new Pose3d( // Climber
                    new Translation3d(0, -0.336, 0.405),
                    new Rotation3d(Units.Degrees.of(-90 + 0), Units.Degrees.of(0), Units.Degrees.of(0)))
        });

        List<Pose3d> coral = SimulatedArena.getInstance().getGamePiecesByType("Coral");

        if (funnel.isHoldingCoral()) {
            Pose3d funnelCoral = funnel.getCoralPose();

            coral.add(funnelCoral);
        }

        if (elevator.isHoldingCoral()) {
            coral.add(elevator.getCoralPose());
        }

        Logger.recordOutput("FieldSimulation/Coral", coral.toArray(new Pose3d[0]));

        List<Pose3d> algae = SimulatedArena.getInstance().getGamePiecesByType("Algae");
        if (intake.isHoldingAlgae()) {
            Pose3d intakeAlgae = intake.getAlgaePosition();
            algae.add(intakeAlgae);
        }
        if (elevator.isHoldingAlgae()) {
            algae.add(elevator.getAlgaePose());
        }
        algae.addAll(fieldManager.getReefAlgae());
        Logger.recordOutput("FieldSimulation/Algae", algae.toArray(new Pose3d[0]));
    }
}
