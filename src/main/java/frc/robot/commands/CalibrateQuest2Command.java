package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CircleFitter;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A command to calibrate the quest's offset from the robot's center of
 * rotation.
 *
 * How it works:
 * 1. The robot slowly rotates in a full circle (360 degrees).
 * 2. While rotating, it records the position of the quest
 * 3. These recorded points should form a circle, where the center of the circle
 * is the robot's center of rotation.
 * 4. After the rotation is complete, a circle-fitting algorithm calculates the
 * center of that circle.
 * 5. The command then calculates the vector from the robot's center to the
 * quest for each data point, transforms that vector into the robot's coordinate
 * frame, and averages the results.
 * 6. The resulting average vector is the quest's physical offset from the
 * robot's center.
 * 
 * Implementation loosely based on:
 * https://github.com/FRC999/2026Prototyping/blob/main/QuestVibeGPT/src/main/java/frc/robot/commands/QuestOffsetCharacterization.java
 */
public class CalibrateQuest2Command extends Command {
    /**
     * A simple data record to store a 2D point from the quest along with the
     * robot's heading at the moment the data was captured.
     *
     * @param position     The (x, y) position of the quest in the world frame
     * @param robotHeading The robot's heading at the time of capture.
     */
    private record PointData(Translation2d position, Rotation2d robotHeading) {
    }

    private static final AngularVelocity ROTATION_SPEED = DegreesPerSecond.of(60);
    private static final int MIN_DATA_POINTS = 100;

    private final SwerveSubsystem swervedrive;
    private final QuestNavSubsystem questnav;
    private final List<PointData> collectedPoints = new ArrayList<>();
    private Rotation2d initialHeading;

    /**
     * Determines the X and Y offset from the center of the robot for the Quest's
     * offset. Spins the robot in a circle and fits the collected data points to a
     * circle and uses the gyro's angle as well as the collected data points to
     * calculate an X and Y offset. The command's status is displayed through
     * the NetworkTable entery {@code QuestCalibratorStatus} and the final X and Y
     * are displayed through the NetworkTable entry {@code QuestOffsetX} and
     * {@code QuestOffsetY} respectively. Note that the X and Y offset are in the
     * native WPILib units of meters.
     * 
     * @param swervedrive the {@link SwerveSubsystem} to rotate in place.
     * @param questnav    the {@link QuestNavSubsystem} to get the position from.
     */
    public CalibrateQuest2Command(SwerveSubsystem swervedrive, QuestNavSubsystem questnav) {
        addRequirements(swervedrive);
        this.swervedrive = swervedrive;
        this.questnav = questnav;
    }

    @Override
    public void initialize() {
        collectedPoints.clear();
        initialHeading = swervedrive.getSwerveDrive().getYaw();
    }

    @Override
    public void execute() {
        // Rotate the robot at the rotation speed
        swervedrive.driveFieldOriented(new ChassisSpeeds(0, 0, ROTATION_SPEED.in(RadiansPerSecond)));

        Optional<Pose2d> questPoseOpt = questnav.getQuestPoseRaw();
        if (questPoseOpt.isPresent()) {
            Translation2d questPosition = questPoseOpt.get().getTranslation();
            Rotation2d currentHeading = swervedrive.getSwerveDrive().getYaw();
            collectedPoints.add(new PointData(questPosition, currentHeading));
        }
        SmartDashboard.putString("QuestCalibratorStatus", "Collecting points: " + collectedPoints.size());
    }

    @Override
    public boolean isFinished() {
        // Check if we have completed a full rotation.
        // The check is for >350 to provide a margin and ensure the command finishes.
        return Math.abs(swervedrive.getSwerveDrive().getYaw().minus(initialHeading).getDegrees()) > 350;
    }

    @Override
    public void end(boolean interrupted) {
        swervedrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0)); // stop the robot

        if (interrupted) {
            DriverStation.reportWarning("Quest calibration interrupted.", false);
            SmartDashboard.putString("QuestCalibratorStatus", "Interrupted. Data discarded.");
            return;
        }

        if (collectedPoints.size() < MIN_DATA_POINTS) {
            DriverStation.reportError("Quest calibration failed: Not enough data points collected ("
                    + collectedPoints.size() + "/" + MIN_DATA_POINTS + ").", false);
            SmartDashboard.putString("QuestCalibratorStatus", "Failed: Not enough data.");
            return;
        }

        SmartDashboard.putString("QuestCalibratorStatus", "Calculating...");

        // 1. Fit a circle to the collected quest positions to find the robot's center
        List<Translation2d> questPositions = collectedPoints.stream().map(PointData::position).toList();

        Optional<Translation2d> robotCenterOpt = CircleFitter.fit(questPositions);

        if (robotCenterOpt.isEmpty()) {
            DriverStation.reportError("Quest calibration failed: Circle fitting algorithm could not find a solution.",
                    false);
            SmartDashboard.putString("QuestCalibratorStatus", "Failed: Circle fit error.");
            return;
        }

        Translation2d robotCenterInWorld = robotCenterOpt.get();

        // 2. Calculate the quest offset in the robot's frame for each point and
        // average them.
        double sumX = 0;
        double sumY = 0;

        for (PointData data : collectedPoints) {
            // Vector from robot center to quest, in the world frame
            Translation2d offsetInWorld = data.position().minus(robotCenterInWorld);

            // Rotate this vector into the robot's frame of reference by applying the
            // inverse of the robot's rotation
            Translation2d offsetInRobot = offsetInWorld.rotateBy(data.robotHeading().unaryMinus());

            sumX += offsetInRobot.getX();
            sumY += offsetInRobot.getY();
        }

        double avgOffsetX = sumX / collectedPoints.size();
        double avgOffsetY = sumY / collectedPoints.size();

        SmartDashboard.putString("QuestCalibratorStatus", "Complete!");
        SmartDashboard.putNumber("QuestOffsetX", avgOffsetX);
        SmartDashboard.putNumber("QuestOffsetY", avgOffsetY);
    }
}
