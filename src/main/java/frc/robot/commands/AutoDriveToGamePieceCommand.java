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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveToGamePieceCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private Supplier<Optional<Pose2d>> goal;
  private PathConstraints constraints;
  private List<Waypoint> waypoints;
  private PathPlannerPath path;
  private Command followingCommand;
  private Alert tooCloseAlert;
  private Alert noObjectAlert;

  /**
   * Auto drives to the closest reef side location. This class essentially wraps
   * {@code AutoBuilder.followPath(path)} with a dynamically changing
   * source/destination.
   * 
   * @param goal            the position to drive to. This should be from the
   *                        blue alliance's AB side in world space coordinates.
   * @param swerveSubsystem the swerve drive to drive the path
   */
  public AutoDriveToGamePieceCommand(Supplier<Optional<Pose2d>> goal, SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.goal = goal;
    constraints = new PathConstraints(2, 1, 1, 1);
    tooCloseAlert = new Alert("Unable to AutoDrive: Too Close!!", AlertType.kWarning);
    noObjectAlert = new Alert("Unable to AutoDrive: No Objects Found!!", AlertType.kWarning);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose = swerveSubsystem.getSwerveDrive().getPose();
    Optional<Pose2d> poseOptional = goal.get();
    Alert activeAlert;

    if (poseOptional.isEmpty()) {
      // nothing to drive to
      activeAlert = noObjectAlert;
    } else {
      activeAlert = tooCloseAlert;
    }
    // this will cause the path to error out below as it'll set the dest to be the
    // current robot's pose
    Pose2d dest = goal.get().orElse(robotPose);

    // the path will error if the drivetrain is too close to the destination
    if (robotPose.getTranslation().getDistance(dest.getTranslation()) < 0.05) {
      // so just do nothing if the path is too close
      DataLogManager.log("Unable to AutoDrive: Too Close");
      activeAlert.set(true);

      // this will infinitly consume the drivetrain, but this command ends once the
      // user releases the trigger (needed for alert to function)
      followingCommand = Commands.idle().finallyDo(() -> activeAlert.set(false));
    } else {
      swerveSubsystem.getSwerveDrive().field.getObject("Goal").setPose(dest);

      // only two waypoints for a smooth path: where we are and where we want to go
      waypoints = PathPlannerPath.waypointsFromPoses(robotPose, dest);
      path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, dest.getRotation()));
      path.preventFlipping = true;

      // add to field for visualization
      swerveSubsystem.getSwerveDrive().field.getObject("traj").setPoses(path.getPathPoses());

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
