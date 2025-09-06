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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePieceDetectorConstants;
import frc.robot.util.objtrack.TrackedGamePieceManager;

public class GamePieceDetector extends SubsystemBase {
	private final PhotonCamera camera;
	private final TrackedGamePieceManager trackedGamePieceManager;
	private final Supplier<Pose2d> robotPoseSupplier;
	private final Field2d field;
	private final Transform2d robotToCameraTransform = new Transform2d(
			GamePieceDetectorConstants.ROBOT_TO_CAMERA.getX(),
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
	 * value represents the directon from the robot's position to the nearest game
	 * piece.
	 * 
	 * @return either the nearest tracked game piece's {@link Pose2d} or
	 *         {@link Optional#empty()} if the robot has not seen any game pieces.
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
				// calculate distance using the algorithm described at
				// https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
				// except we are aiming down so we swap h1 and h2 in the diagram to get a
				// positive distance value
				double gamePiecePitch = Math.tan(Units.degreesToRadians(target.getPitch())
						+ GamePieceDetectorConstants.ROBOT_TO_CAMERA.getRotation().getY());
				if (gamePiecePitch == 0.0) {
					continue; // don't divide by zero
				}
				double camToGamePieceDistance = (GamePieceDetectorConstants.ROBOT_TO_CAMERA.getZ()
						- GamePieceDetectorConstants.OBJECT_HEIGHT_Z.in(Meters)) / gamePiecePitch;

				// now with the distance, rotate the distance value by the camera's yaw value to
				// get the game piece's x and y from the camera (camera space) via the
				// Translation2d's constructor (the constructor multiplies dist by cos/sin of
				// angle for x/y)
				Translation2d gamePieceTranslationCS = new Translation2d(camToGamePieceDistance,
						Rotation2d.fromDegrees(target.getYaw()));

				// convert the translation to a transformation to transform the pose below
				// we don't care about the rotation so just use zero
				Transform2d camToGamePieceTransform = new Transform2d(gamePieceTranslationCS, Rotation2d.kZero);

				// convert from world->robot to robot->camera to camera->game piece
				// to get world->game piece
				Pose2d gamePiecePoseWS = robotPoseSupplier.get()
						.transformBy(robotToCameraTransform)
						.transformBy(camToGamePieceTransform);

				// add to manager
				trackedGamePieceManager.addTrackedGamePiece(gamePiecePoseWS.getTranslation());
			}
		}

		// then update the manager
		trackedGamePieceManager.updateTTL(robotPoseSupplier.get());
		trackedGamePieceManager.updateField(field);
	}
}
