package frc.robot.util.objtrack;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.MutTime;
import frc.robot.Constants.GamePieceDetectorConstants;

public class TrackedGamePiece {
    private final MedianFilter x;
    private final MedianFilter y;
    private final int UUID;
    private final MutTime TTL = Seconds.mutable(0);
    private boolean visibleLatch;

    /**
     * Creates a new game piece to be tracked. Position can be updated with new data
     * via {@link #addIfNear(Translation2d)}. Time to live can be updated through
     * {@link #updateTTL(Pose2d)}.
     * 
     * @param tx   Position of the game piece in the world.
     * @param UUID unique id for the game piece
     */
    public TrackedGamePiece(Translation2d tx, int UUID) {
        this.UUID = UUID;
        x = new MedianFilter(GamePieceDetectorConstants.FILTER_WINDOW_SIZE);
        y = new MedianFilter(GamePieceDetectorConstants.FILTER_WINDOW_SIZE);
        TTL.mut_replace(Seconds.of(0.5));
        visibleLatch = false;

        x.calculate(tx.getX());
        y.calculate(tx.getY());
    }

    /**
     * @return this game piece's unique ID
     */
    public int getUUID() {
        return UUID;
    }

    /**
     * Checks whether the input translation is near this current game piece. If it
     * is, then it'll add to the current game piece and return {@code true}.
     * 
     * @param tx position of new game piece location
     * @return {@code true} if the input translation was near a currently existing
     *         point
     */
    public boolean addIfNear(Translation2d tx) {
        double xx = x.lastValue() - tx.getX();
        double yy = y.lastValue() - tx.getY();
        double distance = Math.sqrt(xx * xx + yy * yy);
        if (distance < GamePieceDetectorConstants.CLOSENESS_THREASHOLD.in(Meters)) {
            x.calculate(tx.getX());
            y.calculate(tx.getY());
            TTL.mut_replace(Seconds.of(0.5));
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets the translation of this tracked game piece on the field.
     * 
     * @return position of tracked game piece on the field.
     */
    public Translation2d getTranslation2d() {
        return new Translation2d(x.lastValue(), y.lastValue());
    }

    /**
     * Updates this tracked game piece's time to live property.
     * 
     * @param robot the current position of the robot in world space. Used to
     *              determine whether this tracked game piece should be in the
     *              camera's FOV to determine TTL.
     * @return {@code true} if this tracked game piece must be removed,
     *         {@code false} if the tracked game piece is still valid.
     */
    public boolean updateTTL(Pose2d robot) {
        // first calculate the view vector from the game piece to the robot
        Translation2d robotTranslationWS = robot.getTranslation(); // where the robot is on the field
        Translation2d gpPositionWS = getTranslation2d(); // where the object is on the field

        // get the angle from the robot to the game piece
        Rotation2d robotToGPAngle = gpPositionWS.minus(robotTranslationWS).getAngle();

        // visible if both looking at the object and near it
        // looking at is calculated by subtracting the game piece angle from the robot
        // to the game piece heading from the robot's heading
        // near is calculated with a simple distance check
        boolean visible = robot.getRotation().minus(robotToGPAngle).getMeasure()
                .abs(Radians) < GamePieceDetectorConstants.CAMERA_FOV.in(Radians)
                && robotTranslationWS.getDistance(gpPositionWS) < GamePieceDetectorConstants.CAMERA_DEPTH.in(Meters);

        // set TTL to 0.5s when in view for the first time
        if (visible && visibleLatch) {
            TTL.mut_replace(Seconds.of(0.5));

            visibleLatch = false;
        } else if (!visible && !visibleLatch) { // set TTL to 15s when out of view

            TTL.mut_replace(Seconds.of(15));
            visibleLatch = true;
        }

        // loop time should be 0.02s
        TTL.mut_minus(Seconds.of(0.02));
        if (TTL.in(Seconds) < 0) {
            return true;
        } else {
            return false;
        }
    }
}
