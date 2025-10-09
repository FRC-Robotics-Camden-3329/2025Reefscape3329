// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class CalibrateQuestCommand {
	// -- Calculate Oculus Offset (copied from
	// https://github.com/5152Alotobots/5152_Reefscape/blob/new_questnav/src/main/java/frc/alotobots/library/subsystems/vision/oculus/commands/OculusCalibration.java
	// --

	private Translation2d calculatedOffsetToRobot = Translation2d.kZero;
	private double calculateOffsetCount = 0;
	private Pose2d initPose = Pose2d.kZero;

	private Translation2d calculateOffsetToRobot(Pose2d questRobotPose) {
		Rotation2d angle = questRobotPose.getRotation().minus(initPose.getRotation());
		Translation2d displacement = questRobotPose.getTranslation().minus(initPose.getTranslation());

		double x = ((angle.getCos() - 1.0) * displacement.getX() + angle.getSin() * displacement.getY())
				/ (2.0 * (1.0 - angle.getCos()));
		double y = ((-angle.getSin()) * displacement.getX() + (angle.getCos() - 1.0) * displacement.getY())
				/ (2.0 * (1.0 - angle.getCos()));
		Translation2d worldFrameOffset = new Translation2d(x, y);

		// To get the offset in the robot's body frame, rotate the world frame
		// offset back by the robot's initial orientation.
		return worldFrameOffset.rotateBy(initPose.getRotation().unaryMinus());
	}

	/**
	 * When calibrating make sure the rotation in your quest transform is right.
	 * (Reality check) If it
	 * is non-zero, you may have to swap the x/y and their signs.
	 */
	public Command determineOffsetToRobotCenter(SwerveSubsystem swerveDrive, QuestNavSubsystem quest) {
		Supplier<Optional<Pose2d>> questPose = quest::getQuestPoseRaw;

		return Commands.repeatingSequence(
				Commands.run(
						() -> {
							// rotate in a circle, can decrease speed to collect more samples
							swerveDrive.driveFieldOriented(
									new ChassisSpeeds(0, 0, Math.PI / 10.0));
						},
						swerveDrive)
						.withTimeout(0.5),
				Commands.runOnce(
						() -> {
							// Update current offset
							Translation2d offset = calculateOffsetToRobot(questPose.get().orElse(Pose2d.kZero));

							// Update average with current offset
							calculatedOffsetToRobot = calculatedOffsetToRobot
									.times((double) calculateOffsetCount
											/ (calculateOffsetCount + 1))
									.plus(offset.div(calculateOffsetCount + 1));
							calculateOffsetCount++;

							SmartDashboard.putNumber("QuestNav/CalculatedOffsetX",
									calculatedOffsetToRobot.getX());
							SmartDashboard.putNumber("QuestNav/CalculatedOffsetY",
									calculatedOffsetToRobot.getY());
							SmartDashboard.putNumber("QuestNav/CalibrationSamples",
									calculateOffsetCount);
						})
						// for numeric stability, we only calculate between 30 degrees and 150 degrees
						.onlyIf(() -> questPose.get().orElse(Pose2d.kZero).getRotation().minus(initPose.getRotation())
								.getDegrees() > 30))
				.until(() -> questPose.get().orElse(Pose2d.kZero).getRotation().minus(initPose.getRotation())
						.getDegrees() > 150)
				.beforeStarting(Commands.runOnce(() -> {
					initPose = quest.getQuestPoseRaw().orElse(initPose);
					calculateOffsetCount = 0;
					calculatedOffsetToRobot = Translation2d.kZero;
				}))
				.finallyDo(
						() -> {
							// Update current offset
							Translation2d offset = calculateOffsetToRobot(questPose.get().orElse(Pose2d.kZero));

							// Update average with current offset
							calculatedOffsetToRobot = calculatedOffsetToRobot
									.times((double) calculateOffsetCount
											/ (calculateOffsetCount + 1))
									.plus(offset.div(calculateOffsetCount + 1));
							calculateOffsetCount++;

							SmartDashboard.putNumber("QuestNav/CalculatedOffsetX",
									calculatedOffsetToRobot.getX());
							SmartDashboard.putNumber("QuestNav/CalculatedOffsetY",
									calculatedOffsetToRobot.getY());
							SmartDashboard.putNumber("QuestNav/CalibrationSamples",
									calculateOffsetCount);
						});
	}
}
