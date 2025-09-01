// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveToGamePieceCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Optional<Pose2d>> goalSupplier;
    private final PathConstraints constraints;
    private final Alert tooCloseAlert;
    private final Alert noGPAlert;
    private List<Waypoint> waypoints;
    private PathPlannerPath path;
    private Command followingCommand;

    /**
     * Auto drives to the closest game piece. This class essentially wraps
     * {@link AutoBuilder#followPath(PathPlannerPath)} with a dynamically changing
     * goal based on the nearest game object supplied in {@code goalSupplier}.
     * <p>
     * Will alert if the requested pose to drive to is too close or if there are no
     * valid game pieces to drive to.
     * 
     * @param goalSupplier    the position to drive to in worldspace coordinates.
     *                        Will not drive if the optional supplied
     *                        {@link Optional#isEmpty()} (assumed to have no valid
     *                        game pieces to drive to).
     * @param swerveSubsystem the swerve drive to drive the path
     */
    public AutoDriveToGamePieceCommand(Supplier<Optional<Pose2d>> goalSupplier, SwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.goalSupplier = goalSupplier;
        constraints = new PathConstraints(2, 1, 1, 1);
        tooCloseAlert = new Alert("Unable to AutoDrive: Too Close!!", AlertType.kWarning);
        noGPAlert = new Alert("Unable to AutoDrive: No Game Pieces Found!!", AlertType.kWarning);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d robotPose = swerveSubsystem.getSwerveDrive().getPose();
        Optional<Pose2d> goalPoseOptional = goalSupplier.get();
        Alert activeAlert;

        // change the alert to prioritize no game pieces being found
        if (goalPoseOptional.isEmpty()) {
            // nothing to drive to
            activeAlert = noGPAlert;
        } else {
            activeAlert = tooCloseAlert;
        }

        // this will make the goal position either the nearest game piece if one exists
        // or the current robot's position if there isn't any game pices. We do this
        // because it will error out on the closeness check to prevent the command from
        // running if there are no game pieces to drive to
        Pose2d dest = new Pose2d(goalPoseOptional.orElse(robotPose).getTranslation(), robotPose.getRotation());

        // the path will error if the drivetrain is too close to the destination
        if (robotPose.getTranslation().getDistance(dest.getTranslation()) < 0.05) {
            // so just do nothing if the path is too close
            activeAlert.set(true);

            // this will infinitly consume the drivetrain, but this command ends once the
            // user releases the trigger (needed for alert to function)
            followingCommand = Commands.idle().finallyDo(() -> activeAlert.set(false));
        } else {
            swerveSubsystem.getSwerveDrive().field.getObject("GoalGP").setPose(dest);

            // only two waypoints for a smooth path: where we are and where we want to go
            waypoints = PathPlannerPath.waypointsFromPoses(robotPose, dest);
            // we'll set the goal's ending heading to be the same as the robot is currently
            // to move the robot forward without turning
            path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, robotPose.getRotation()));
            path.preventFlipping = true;

            // add to field for visualization
            swerveSubsystem.getSwerveDrive().field.getObject("trajGP").setPoses(path.getPathPoses());

            followingCommand = AutoBuilder.followPath(path);
        }
        followingCommand.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        followingCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        followingCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return followingCommand.isFinished();
    }
}
