// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestConstants;
import frc.robot.util.VisionData.EstimateConsumer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
	private final QuestNav questNav = new QuestNav();
	private final Alert disconnectedAlert;
	private final EstimateConsumer estConsumer;
	private final StructPublisher<Pose2d> worldPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("QuestNav/WorldPose", Pose2d.struct).publish();
	private Optional<Pose2d> questWorldPose = Optional.empty();
	private boolean useEstConsumer;

	/** Creates a new QuestNavSubsystem. */
	public QuestNavSubsystem(EstimateConsumer estimateConsumer, boolean useEstConsumer) {
		this.estConsumer = estimateConsumer;
		this.useEstConsumer = useEstConsumer;
		disconnectedAlert = new Alert("QuestNav Not Tracking!!", AlertType.kWarning);
	}

	/**
	 * Sets whether the QuestNav subsystem should be using the estimation
	 * consumer provided in the constructor. If true, then this will update the
	 * robot's position based on QuestNav's pose estimation.
	 */
	public void setUseEstConsumer(boolean useEstConsumer) {
		this.useEstConsumer = useEstConsumer;
	}

	/**
	 * @return whether questnav is currently tracking. Note this is different from
	 *         being connected.
	 */
	public boolean isTracking() {
		return questNav.isTracking();
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
		setQuestPoseRaw(questPose);
	}

	/**
	 * Sets the quest pose without transforming by the robot to quest offset.
	 * 
	 * @param pose pose to set the quest to.
	 */
	public void setQuestPoseRaw(Pose2d pose) {
		if (questNav.isConnected()) {
			questNav.setPose(pose);
		} else {
			DriverStation.reportError(
					"Trying to set Quest position but it is not connected!! (is it connected?)",
					true);
		}
	}

	/**
	 * Gets the position of the quest in worldspace. Applies the nessesary robot to
	 * quest transformation.
	 * 
	 * @return The Quest's worldspace position. Will be None if not tracking.
	 */
	public Optional<Pose2d> getQuestPose() {
		return getQuestPoseRaw().map(pose -> pose.transformBy(QuestConstants.ROBOT_TO_QUEST.inverse()));
	}

	/**
	 * @return Gets the quest pose without any transformation. Will be None if not
	 *         tracking.
	 */
	public Optional<Pose2d> getQuestPoseRaw() {
		periodic(); // update and make sure we have the latest quest world pose
		if (questWorldPose.isEmpty()) {
			DriverStation.reportError(
					"Trying to get Quest pose but it is not avaliable!! (is the Quest connected and tracking?)",
					true);
		}
		return questWorldPose;

		// The below is how you would get the world pose if the world pose was not being
		// continually updated in the subsystem's periodic method. The periodic method
		// will automatically fetch the latest pose. We cannot get the pose here because
		// the periodic method will empty the quest's internal queue, returning no data
		// here. Only uncomment the below if not using the subsystem's periodic method
		// to fetch the quest pose.

		// PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

		// // first check if there is data availiable
		// if (poseFrames.length > 0) {
		// // Get the most recent Quest pose
		// return Optional.of(poseFrames[poseFrames.length - 1].questPose());
		// } else {
		// // there is no data avaliable
		// DriverStation.reportWarning("Trying to access quest position but there is no
		// data!!", false);
		// return Optional.empty();
		// }
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		questNav.commandPeriodic();
		disconnectedAlert.set(!questNav.isTracking());

		if (questNav.isTracking()) {
			// Get the latest pose data frames from the Quest
			PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

			// Loop over the pose data frames and send them to the pose estimator
			for (PoseFrame questFrame : questFrames) {
				// Get the pose of the Quest
				Pose2d questPose = questFrame.questPose();
				questWorldPose = Optional.of(questPose);
				// Get timestamp for when the data was sent
				double timestamp = questFrame.dataTimestamp();

				// Transform by the mount pose to get robot pose
				Pose2d robotPose = questPose.transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());

				// add quest position to be tracked seperately from the robot's estimator
				worldPosePublisher.accept(robotPose);

				// Add the measurement to the estimator
				if (useEstConsumer) {
					estConsumer.accept(robotPose, timestamp, QuestConstants.QUESTNAV_STD_DEVS);
				}
			}

		} else {
			questWorldPose = Optional.empty();
		}

		SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
		SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
		SmartDashboard.putNumber("QuestNav/Quest Battery", questNav.getBatteryPercent().orElse(-1));
	}

}
