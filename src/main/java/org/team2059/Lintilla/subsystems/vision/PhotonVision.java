package org.team2059.Lintilla.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2059.Lintilla.Constants;

import java.util.List;
import java.util.Optional;

public class PhotonVision extends SubsystemBase {
	// Cental camera object
	private final PhotonCamera camera;

	// Store all data that PhotonVision returns
	private List<PhotonPipelineResult> cameraResults;

	// Store latest data that PhotonVision returns
	private PhotonPipelineResult cameraResult;

	// PhotonPoseEstimators, one for each pipeline
	private final PhotonPoseEstimator poseEstimator;

	private Matrix<N3, N1> currentStdDevs;

	private Optional<Pose3d> estimatedPose;

	public PhotonVision() {
		camera = new PhotonCamera(Constants.VisionConstants.pvCamName);

		poseEstimator = new PhotonPoseEstimator(
		  Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT,
		  Constants.VisionConstants.ROBOT_TO_PV
		);

		estimatedPose = Optional.empty();
	}

	private void updateEstimationStdDevs(
	  Optional<EstimatedRobotPose> estimatedPose,
	  List<PhotonTrackedTarget> targets
	) {
		if (estimatedPose.isEmpty()) {
			// No pose input. Default to single-tag std devs
			currentStdDevs = Constants.VisionConstants.singleTagStdDevsPV;
		} else {
			// Pose present. Start running heuristic.
			var estStdDevs = Constants.VisionConstants.singleTagStdDevsPV;
			int numTags = 0;
			double avgDist = 0;

			// Precalculation - see how many tags we cound, and calculate an average-distance metric
			for (var tgt : targets) {
				var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
				if (tagPose.isEmpty()) continue;
				numTags++;
				avgDist +=
				  tagPose
					.get()
					.toPose2d()
					.getTranslation()
					.getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
			}

			if (numTags == 0) {
				// No tags visible. Default to single-tag std devs
				currentStdDevs = Constants.VisionConstants.singleTagStdDevsPV;
			} else {
				// One or more tags visible, run the full heuristic.
				avgDist /= numTags;
				// Increase std devs if multiple targets are visible.
				if (numTags > 1) estStdDevs = Constants.VisionConstants.multiTagStdDevsPV;
				// Increase std devs based on (average) distance
				if (numTags == 1 && avgDist > 4) {
					estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				} else {
					estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
				}
				currentStdDevs = estStdDevs;
			}
		}
	}

	public Optional<Pose3d> getEstimatedPose() {
		return estimatedPose;
	}

	public Matrix<N3, N1> getCurrentStdDevs() {
		return currentStdDevs;
	}

	@Override
	public void periodic() {
		// Log values
		Logger.recordOutput("PhotonVision/Connected", camera.isConnected());

		cameraResults = camera.getAllUnreadResults();

		if (!cameraResults.isEmpty()) {
			cameraResult = cameraResults.get(cameraResults.size() - 1);

			Logger.recordOutput("PhotonVision/HasTarget", cameraResult.hasTargets());

			if (cameraResult.hasTargets()) {
				Logger.recordOutput("PhotonVision/BestTargetID", cameraResult.getBestTarget().getFiducialId());
			} else {
				Logger.recordOutput("PhotonVision/BestTargetID", -1);
			}
		}

		Optional<EstimatedRobotPose> visionEst = Optional.empty();
		for (var change : cameraResults) {
			// Attempt multi-tag estimation
			visionEst = poseEstimator.estimateCoprocMultiTagPose(change);

			// If multi-tag fails, fall back to lowest ambiguity
			if (visionEst.isEmpty()) {
				visionEst = poseEstimator.estimateLowestAmbiguityPose(change);
			}

			// Process standard deviations
			updateEstimationStdDevs(visionEst, change.getTargets());
		}

		if (visionEst.isPresent()) {
			estimatedPose = Optional.of(visionEst.get().estimatedPose);
		} else {
			estimatedPose = Optional.empty();
		}

		Logger.recordOutput("PhotonVision/EstimatedPose", estimatedPose.isPresent() ? estimatedPose.get() : null);
	}
}
