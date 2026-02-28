package org.team2059.Lintilla.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.Constants;
import org.team2059.Lintilla.RobotContainer;

/**
 * Wrapper for the QuestNav vendor dependency
 */
public class Oculus extends SubsystemBase {
	private final QuestNav questNav;

	private boolean useMeasurements;

	private Pose3d robotPose;
	private Pose3d rawPose;

	public Oculus() {
		questNav = new QuestNav();

		useMeasurements = !RobotContainer.buttonBox.getRawButton(Constants.OperatorConstants.ButtonBoxQuestMeasurement);
		;

		robotPose = null;
		rawPose = null;
	}

	/**
	 * @return the current estimated Pose3d WITH robot transform applied
	 */
	public Pose3d getRobotPose() {
		return robotPose;
	}

	/**
	 * Set the Quest-reported ROBOT pose. Offset applied automatically. Where do you want the robot to think it is?
	 *
	 * @param pose the Pose3d to set to
	 */
	public void setRobotPose(Pose3d pose) {
		questNav.setPose(pose.transformBy(Constants.VisionConstants.ROBOT_TO_QUEST));
	}

	/**
	 * Pose2d version of this method. All other values set to zero. Check whether you need 3d positioning data.
	 *
	 * @param pose the Pose2d to set to
	 */
	public void setRobotPose(Pose2d pose) {
		questNav.setPose(new Pose3d(pose).transformBy(Constants.VisionConstants.ROBOT_TO_QUEST));
	}

	/**
	 * @return the current estimated Pose3d WITHOUT robot transform applied
	 */
	public Pose3d getRawPose() {
		return rawPose;
	}

	/**
	 * Set the raw Quest pose, with NO robot offsets included.
	 *
	 * @param pose the Pose3d to set to
	 */
	public void setRawPose(Pose3d pose) {
		questNav.setPose(pose);
	}

	/**
	 * @return whether measurements from the Quest are currently being used
	 */
	public boolean isUsingMeasurements() {
		return isUsingMeasurements();
	}

	/**
	 * Set whether measurements are being used for pose estimation
	 */
	public void setUseMeasurements(boolean b) {
		useMeasurements = b;
	}

	@Override
	public void periodic() {
		// Run periodic function
		questNav.commandPeriodic();

		// Log values
		Logger.recordOutput("Oculus/Connected", questNav.isConnected());
		Logger.recordOutput("Oculus/Tracking", questNav.isTracking());
		Logger.recordOutput("Oculus/LostTrackingCount", questNav.getTrackingLostCounter().isPresent() ? questNav.getTrackingLostCounter().getAsInt() : -1);
		Logger.recordOutput("Oculus/BatteryPercent", questNav.getBatteryPercent().isPresent() ? questNav.getBatteryPercent().getAsInt() : -1);
		Logger.recordOutput("Oculus/Latency", questNav.getLatency());
		Logger.recordOutput("Oculus/UsingOculusMeasurements", useMeasurements);

		// Get the latest pose data frames from the Quest
		PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

		// Add measurements only if certain conditions are met
		if (questNav.isConnected() && questNav.isTracking() && useMeasurements) {
			// Loop over pose data frames, send to pose estimator
			for (PoseFrame questFrame : questFrames) {
				// Make sure Quest was tracking for this frame
				if (questFrame.isTracking()) {
					// Get the pose of the Quest
					rawPose = questFrame.questPose3d();

					// Get timestamp for when data was sent
					double timestamp = questFrame.dataTimestamp();

					// Transform by the mount pose to get robot pose
					robotPose = rawPose.transformBy(Constants.VisionConstants.ROBOT_TO_QUEST.inverse());

					// TODO: filtering?

					// Add measurement to pose estimator
					RobotContainer.drivetrain.addVisionMeasurement(
					  robotPose.toPose2d(),
					  timestamp,
					  Constants.VisionConstants.questNavStdDevs
					);
				}
			}
		}

		if (rawPose != null) {
			Logger.recordOutput("Oculus/RawPose", rawPose);
		}
		if (robotPose != null) {
			Logger.recordOutput("Oculus/RobotPose", robotPose);
		}
	}
}
