// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

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
	private double calculateOffsetCount = 1;
	private Pose2d initPose;

	private Translation2d calculateOffsetToRobot(Pose2d questRobotPose) {
		Rotation2d angle = questRobotPose.getRotation().minus(initPose.getRotation());
		Translation2d displacement = questRobotPose.getTranslation().minus(initPose.getTranslation());

		double x = ((angle.getCos() - 1.0) * displacement.getX() + angle.getSin() * displacement.getY())
				/ (2.0 * (1.0 - angle.getCos()));
		double y = ((-angle.getSin()) * displacement.getX() + (angle.getCos() - 1.0) * displacement.getY())
				/ (2.0 * (1.0 - angle.getCos()));

		return new Translation2d(x, y);
	}

	/**
	 * When calibrating make sure the rotation in your quest transform is right.
	 * (Reality check) If it
	 * is non-zero, you may have to swap the x/y and their signs.
	 */
	public Command determineOffsetToRobotCenter(SwerveSubsystem swerveDrive, QuestNavSubsystem quest) {
		// First reset our pose to 0, 0
		// quest.setQuestPoseRaw(Pose2d.kZero);
		initPose = quest.getQuestPoseRaw();
		Supplier<Pose2d> questPose = quest::getQuestPoseRaw;
		return Commands.repeatingSequence(
				Commands.run(
						() -> {
							swerveDrive.driveFieldOriented(
									new ChassisSpeeds(0, 0, Math.PI / 10.0));
						},
						swerveDrive)
						.withTimeout(0.5),
				Commands.runOnce(
						() -> {
							// Update current offset
							Translation2d offset = calculateOffsetToRobot(questPose.get());
							calculatedOffsetToRobot = calculatedOffsetToRobot
									.times((double) calculateOffsetCount
											/ (calculateOffsetCount + 1))
									.plus(offset.div(calculateOffsetCount + 1));
							calculateOffsetCount++;

							SmartDashboard.putNumber("OculusCalibration/CalculatedOffsetX",
									calculatedOffsetToRobot.getX());
							SmartDashboard.putNumber("OculusCalibration/CalculatedOffsetY",
									calculatedOffsetToRobot.getY());
						})
						.onlyIf(() -> questPose.get().getRotation().getDegrees() > 30))
				.finallyDo(
						() -> {
							// Update current offset
							Translation2d offset = calculateOffsetToRobot(questPose.get());
							calculatedOffsetToRobot = calculatedOffsetToRobot
									.times((double) calculateOffsetCount
											/ (calculateOffsetCount + 1))
									.plus(offset.div(calculateOffsetCount + 1));
							calculateOffsetCount++;

							SmartDashboard.putNumber("OculusCalibration/CalculatedOffsetX",
									calculatedOffsetToRobot.getX());
							SmartDashboard.putNumber("OculusCalibration/CalculatedOffsetY",
									calculatedOffsetToRobot.getY());
						});
	}
}
