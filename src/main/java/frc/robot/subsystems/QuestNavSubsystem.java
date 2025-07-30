// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestConstants;
import frc.robot.util.VisionData;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
	private QuestNav questNav = new QuestNav();
	private VisionData visionData = new VisionData(null, -1, null);

	private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("QuestNav/WorldPose", Pose2d.struct).publish();

	/** Creates a new QuestNavSubsystem. */
	public QuestNavSubsystem() {
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
		// Get the Quest pose
		Pose2d questPose = questNav.getPose();

		// Transform by the offset to get your final pose!
		Pose2d robotPose = questPose.transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());
		return robotPose;
	}

	/**
	 * @return Gets the quest pose without any transformation or tracking/connection
	 *         checks.
	 */
	public Pose2d getQuestPoseRaw() {
		return questNav.getPose();
	}

	public VisionData getVisionData() {
		if (questNav.isConnected() && questNav.isTracking()) {
			Pose2d pose = getQuestPose();
			// Get timestamp from the QuestNav instance
			double timestamp = questNav.getDataTimestamp();

			SmartDashboard.putNumber("QuestNav/Pose X", pose.getX());
			SmartDashboard.putNumber("QuestNav/Pose Y", pose.getY());
			publisher.set(pose);
			visionData.updateVisionData(pose, timestamp, QuestConstants.QUESTNAV_STD_DEVS);
		} else {
			SmartDashboard.putNumber("QuestNav/Pose X", 3329);
			SmartDashboard.putNumber("QuestNav/Pose Y", 3329);
			visionData.updateVisionData(null, -1, null);
		}
		return visionData;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		questNav.commandPeriodic();
		SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
		SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
		SmartDashboard.putNumber("QuestNav/Quest Battery", questNav.getBatteryPercent());
	}

}
