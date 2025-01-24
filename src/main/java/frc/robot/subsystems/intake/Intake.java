package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.generated.TunerConstants;

public class Intake extends SubsystemBase {

    private double currentPosition = 0;
    private double desiredPosition = 0;

    private Goal currentGoal = Goal.STOW;

    public enum Goal {
        INTAKING,
        STOW
    }

    public Intake() {

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
            double delta = Math.min(Math.abs(difference),5);
            currentPosition += Math.signum(difference) * delta;
        }

        Logger.recordOutput("Intake/goal", currentGoal);
        Logger.recordOutput("Intake/current", currentPosition);
        Logger.recordOutput("Intake/desired", desiredPosition);
    }

    public void toggleIntakePosition() {
        if (currentGoal.equals(Goal.INTAKING)) {
            currentGoal = Goal.STOW;
        } else {
            currentGoal = Goal.INTAKING;
        }
    }

    public Pose3d getPose() {
        return new Pose3d( // Algae Intake
                    // new Translation3d(0, 0, 0),
                    new Translation3d(-0.31, 0, 0.19),
                    new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(-150 + currentPosition), Units.Degrees.of(180)));
    }



}
