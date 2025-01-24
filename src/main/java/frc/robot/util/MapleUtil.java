package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

public class MapleUtil {
    public static GamePieceProjectile generateCoralAtPoint(Pose3d initialPose, Translation3d initialVelocity) {
        return new GamePieceProjectile(
                        ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                        initialPose.toPose2d().getTranslation(),
                        initialVelocity.toTranslation2d(),
                        initialPose.getZ(),
                        initialVelocity.getZ(),
                        initialPose.getRotation())
                .enableBecomesGamePieceOnFieldAfterTouchGround()
                .withTouchGroundHeight(0.2);
    }
}
