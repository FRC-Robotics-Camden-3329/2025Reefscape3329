// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestConstants;
import frc.robot.util.VisionData.EstimateConsumer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
	private final QuestNav questNav = new QuestNav();
	private final EstimateConsumer estConsumer;
	private final StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("QuestNav/WorldPose", Pose2d.struct).publish();

	/** Creates a new QuestNavSubsystem. */
	public QuestNavSubsystem(EstimateConsumer estimateConsumer) {
		this.estConsumer = estimateConsumer;
	}

	/**
	 * Sets the Quest's position in worldspace coordinates. Applies the nessesary
	 * robot to quest transformation.
	 * 
	 * @param pose The position in worldspace.
	 */
	public void setQuestPose(Pose2d pose) {
		// Transform by the offset to get the Quest pose
		Pose2d questPose = pose.transformBy(QuestConstants.ROBOT_TO_QUEST);

		// Send the reset operation
		questNav.setPose(questPose);
	}

	/**
	 * Sets the quest pose without transforming by the robot to quest offset.
	 * 
	 * @param pose pose to set the quest to.
	 */
	public void setQuestPoseRaw(Pose2d pose) {
		questNav.setPose(pose);
	}

	/**
	 * Gets the position of the quest in worldspace. Applies the nessesary robot to
	 * quest transformation.
	 * 
	 * <p>
	 * <b>note: this does not check if the quest is connected and tracking</b>, use
	 * the
	 * supplier to get the position to check if it is connected and tracking.
	 * 
	 * @return The Quest's worldspace position.
	 */
	public Pose2d getQuestPose() {
		Pose2d robotPose = getQuestPoseRaw().transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());
		return robotPose;
	}

	/**
	 * @return Gets the quest pose without any transformation or tracking/connection
	 *         checks.
	 */
	public Pose2d getQuestPoseRaw() {
		// return questNav.getPose();
		PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

		// first check if there is data availiable
		if (poseFrames.length > 0) {
			// Get the most recent Quest pose
			Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();
			// Transform by the mount pose to get your robot pose
			return questPose;
		} else {
			return new Pose2d(3329, 3329, Rotation2d.kZero);
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		questNav.commandPeriodic();
		if (questNav.isTracking()) {
			// Get the latest pose data frames from the Quest
			PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

			// Loop over the pose data frames and send them to the pose estimator
			for (PoseFrame questFrame : questFrames) {
				// Get the pose of the Quest
				Pose2d questPose = questFrame.questPose();
				// Get timestamp for when the data was sent
				double timestamp = questFrame.dataTimestamp();

				// Transform by the mount pose to get your robot pose
				Pose2d robotPose = questPose.transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());

				// add quest position to be tracked seperately from the robot's fused position
				publisher.accept(robotPose);

				// Add the measurement to our estimator
				estConsumer.accept(robotPose, timestamp, QuestConstants.QUESTNAV_STD_DEVS);
			}

			SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
			SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
			SmartDashboard.putNumber("QuestNav/Quest Battery",
					questNav.getBatteryPercent().orElse(-1));
		}
	}

}
