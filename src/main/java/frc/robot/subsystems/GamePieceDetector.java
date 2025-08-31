// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePieceDetectorConstants;
import frc.robot.util.objtrack.TrackedGamePieceManager;

public class GamePieceDetector extends SubsystemBase {
	private final PhotonCamera camera;
	private final TrackedGamePieceManager trackedGamePieceManager;
	private final Supplier<Pose2d> robotPoseSupplier;
	private final Field2d field;
	private final Transform2d camTransform = new Transform2d(GamePieceDetectorConstants.ROBOT_TO_CAMERA.getX(),
			GamePieceDetectorConstants.ROBOT_TO_CAMERA.getY(),
			GamePieceDetectorConstants.ROBOT_TO_CAMERA.getRotation().toRotation2d());

	/**
	 * Stores a history of seen game pieces and lets you get the nearest one's
	 * position with {@link #getNearestGamePiecePose()}.
	 * 
	 * @param robotPoseSupplier a supplier to get the current robot's world position
	 * @param field             a field to add tracked game pieces onto for
	 *                          visualization purposes
	 */
	public GamePieceDetector(Supplier<Pose2d> robotPoseSupplier, Field2d field) {
		this.robotPoseSupplier = robotPoseSupplier;
		this.field = field;
		camera = new PhotonCamera(GamePieceDetectorConstants.CAMERA_NAME);
		trackedGamePieceManager = new TrackedGamePieceManager();

	}

	/**
	 * Gets the nearest game piece to the robot. The object's rotation
	 * value has no real meaning here, but {@link Pose2d}s are easier to work with
	 * later.
	 * 
	 * @return either the nearest tracked game piece's {@link Pose2d} or
	 *         {@code Optional.Empty} if the robot has not seen any game pieces.
	 */
	public Optional<Pose2d> getNearestGamePiecePose() {
		return trackedGamePieceManager.getNearestGamePiecePose(robotPoseSupplier.get());
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
		// first add any new game pieces to the mamanger
		for (var result : camera.getAllUnreadResults()) {
			for (var target : result.getTargets()) {
				// convert camera pitch and yaw to x and y coordinates relative to the camera
				// through spherical to Cartesian coordinate transformations
				double r = GamePieceDetectorConstants.OBJECT_HEIGHT_Z.in(Meters) / Math.cos(target.getYaw());
				double theta = target.getYaw() + GamePieceDetectorConstants.ROBOT_TO_CAMERA.getRotation().getZ();
				double rho = target.getPitch() + GamePieceDetectorConstants.ROBOT_TO_CAMERA.getRotation().getY();
				double x = r * Math.sin(theta) * Math.cos(rho);
				double y = r * Math.sin(theta) * Math.sin(rho);

				// the rotation value is irrelavent here
				Pose2d targ = new Pose2d(x, y, Rotation2d.kZero);

				// coordinate transformation: camera to robot
				targ = targ.transformBy(camTransform.inverse());

				// coordinate transformation: robot to world (not 100% sure this is needed)
				targ = targ.relativeTo(robotPoseSupplier.get());

				// add to manager
				trackedGamePieceManager.addTrackedGamePiece(targ.getTranslation());
			}
		}

		// then update the manager
		trackedGamePieceManager.updateTTL(robotPoseSupplier.get());
		trackedGamePieceManager.updateField(field);
	}
}
