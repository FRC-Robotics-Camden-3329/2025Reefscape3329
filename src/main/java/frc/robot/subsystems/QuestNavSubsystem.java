// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QuestConstants;
import frc.robot.util.VisionData;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav = new QuestNav();

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
  private Pose2d getQuestPose() {
    // Get the Quest pose
    Pose2d questPose = questNav.getPose();

    // Transform by the offset to get your final pose!
    Pose2d robotPose = questPose.transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());
    return robotPose;
  }

  public VisionData getVisionData() {
    if (questNav.isConnected() && questNav.isTracking()) {
      Pose2d pose = getQuestPose();
      // Get timestamp from the QuestNav instance
      double timestamp = questNav.getDataTimestamp();

      // field.getObject("quest").setPose(pose);
      // You can put some sort of filtering here if you would like!

      // Add the measurement to our estimator
      // swerveDrive.addVisionMeasurement(pose, timestamp, QUESTNAV_STD_DEVS);
      SmartDashboard.putNumber("QN Pose X", pose.getX());
      SmartDashboard.putNumber("QN Pose Y", pose.getY());
      return new VisionData(pose, timestamp, QuestConstants.QUESTNAV_STD_DEVS);
    } else {
      SmartDashboard.putNumber("QN Pose X", -1.0d);
      SmartDashboard.putNumber("QN Pose Y", -1.0d);
      return new VisionData(null, -1.0, QuestConstants.QUESTNAV_STD_DEVS);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    questNav.commandPeriodic();
    SmartDashboard.putBoolean("QN connected", questNav.isConnected());
    SmartDashboard.putBoolean("QN tracking", questNav.isTracking());
    SmartDashboard.putNumber("Quest Battery", questNav.getBatteryPercent());
  }

}
