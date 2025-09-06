package frc.robot.util.objtrack;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class TrackedGamePieceManager {
    private final List<TrackedGamePiece> trackedGamePieces;
    private int uuidCount;

    /**
     * A manager to remember seen game pieces on the field.
     */
    public TrackedGamePieceManager() {
        trackedGamePieces = new ArrayList<>();
        uuidCount = 0;
    }

    /**
     * Adds or updates a game piece. If the new game piece is near an already
     * existing tracked game piece, it will update the position of the already
     * tracked game piece.
     * 
     * @param tx game piece to add
     */
    public void addTrackedGamePiece(Translation2d tx) {
        // check if it is empty, if it is then bypass checking the list entirely
        if (!trackedGamePieces.isEmpty()) {
            for (var trackedGamePiece : trackedGamePieces) {
                if (trackedGamePiece.addIfNear(tx)) {
                    // when the add is successfull, we are done here
                    return;
                }
            }
        }
        trackedGamePieces.add(new TrackedGamePiece(tx, uuidCount++));
    }

    /**
     * Updates each tracked game piece's time to live. This is done to ensure any
     * erroneously tracked game piece will expire and not be remembered. The time to
     * live is 0.5s for any game piece in the field of view of the camera and 15s
     * for any game piece not in the field of view of the camera.
     * 
     * @param pose the robot's current world space position on the field. Required
     *             to calculate whether the tracked game piece is in the camera's
     *             FOV.
     */
    public void updateTTL(Pose2d pose) {
        trackedGamePieces.removeIf(tgp -> tgp.updateTTL(pose));
    }

    /**
     * Gets the nearest game piece to the supplied pose. This
     * {@link Pose2d#getRotation()} value represents the angle from the input pose
     * to the tracked game piece.
     * 
     * @param pose pose to get the nearest game piece from, probably the robot's
     *             current position.
     * @return {@link Optional#of(Pose2d)} if the robot has a tracked game
     *         piece or {@link Optional#empty()} if the robot has no tracked game
     *         pieces.
     */
    public Optional<Pose2d> getNearestGamePiecePose(Pose2d pose) {
        if (trackedGamePieces.isEmpty()) {
            return Optional.empty();
        }

        /**
         * Uses `Pose2d.nearest(List<Pose2d>)` to filter for the nearest position. We
         * map the internal `List<TrackedGamePiece>` to `List<Pose2d>` by using the game
         * piece's `Translation2d` and calculating a new `Rotation2d` value from the
         * angle between the input pose to the game piece.
         */
        return Optional.of(
                pose.nearest(trackedGamePieces.stream().map(tgp -> new Pose2d(tgp.getTranslation2d(),
                        tgp.getTranslation2d().minus(pose.getTranslation()).getAngle())).toList()));
    }

    /**
     * Updates the field with all the currently tracked game pieces. Primarily used
     * for debug but could be used as a driver aid.
     * <p>
     * Tracked game piece pointing up: in camera view
     * <p>
     * Tracked game piece pointing down: not in camera view
     * 
     * @param field the field to populate objects onto
     */
    public void updateField(Field2d field) {
        trackedGamePieces.forEach(
                tgp -> field.getObject("Game Piece " + tgp.getUUID())
                        .setPose(tgp.getPose2d()));
    }

}
