package org.team2059.Lintilla.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2059.Lintilla.Constants;
import org.team2059.Lintilla.RobotContainer;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;

import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;

import static edu.wpi.first.units.Units.Meters;
import static org.team2059.Lintilla.Constants.OperatorConstants.PHOTONVISION_MEASUREMENT_SWITCH;
import static org.team2059.Lintilla.Constants.OperatorConstants.QUEST_MEASUREMENT_SWITCH;
import static org.team2059.Lintilla.Constants.VisionConstants.*;

/**
 * Subsystem for the localization system.
 * <p>
 * This subsystem integrates PhotonVision and QuestNav vision systems to provide pose estimation and field localization.
 * Vision measurements are fused from multiple sources and published for use by other subsystems.
 */
public class LocalizationSystem extends SubsystemBase {

    private static LocalizationSystem instance;

    private final QuestNav questNav;

    private final PhotonCamera pvCam;
    private final PhotonCamera pvCam2;

    private final PhotonPoseEstimator pvEstimator1;
    private final PhotonPoseEstimator pvEstimator2;

    private boolean qnavConnected = false;
    private boolean qnavTracking = false;
    private int qnavLostTrackingCount = -1;
    private int qnavBatteryPercent = -1;
    private double qnavLatency = -1;

    private Pose3d qnavRobotPose = new Pose3d();
    private Pose3d qnavRawPose = new Pose3d();

    private boolean qnavUseMeasurements;

    private double qnavFaultCounter = 0.0;
    private boolean qnavHealthy = false;

    private Matrix<N3, N1> pvStdDevs1;
    private Matrix<N3, N1> pvStdDevs2;


    private Pose3d pvRobotPose1 = new Pose3d();
    private Pose3d pvRobotPose2 = new Pose3d();
    private Pose3d pvAvgRobotPose = new Pose3d();

    private boolean pvUseMeasurements;
    private boolean pvConnected = false;
    private boolean pvHasTarget = false;
    private int pvBestTargetId = -1;

    private double pvWeight = 0.0;
    private double pvWeight2 = 0.0;

    /**
     * Constructor for LocalizationSystem
     * <p>
     * Configures QuestNav and PhotonVision parts
     */
    private LocalizationSystem() {

        // Set up QuestNav
        questNav = new QuestNav();

        qnavUseMeasurements =
            !RobotContainer.buttonBox.getRawButton(QUEST_MEASUREMENT_SWITCH);

        // Set up PhotonVision cameras and estimators
        pvCam = new PhotonCamera(PV_CAM_NAME);
        pvCam2 = new PhotonCamera(PV_CAM_NAME_2);

        pvEstimator1 = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            LEFT_ROBOT_TO_PV
        );

        pvEstimator2 = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            RIGHT_ROBOT_TO_PV
        );

        pvUseMeasurements =
            !RobotContainer.buttonBox.getRawButton(PHOTONVISION_MEASUREMENT_SWITCH);

        // Don't flood the console with "camera not detected" messages
        PhotonCamera.setVersionCheckEnabled(false);
    }

    public static LocalizationSystem getInstance() {
        if (instance == null) {
            throw new RuntimeException(
                "LocalizationSystem is not initialized! Call initialize() first"
            );
        }

        return instance;
    }

    public static void initialize() {
        if (instance == null) {
            instance = new LocalizationSystem();
        }
    }

    /**
     * @return whether QuestNav headset is connected
     */
    public boolean isQnavConnected() {
        return qnavConnected;
    }

    /**
     * @return whether QuestNav headset is tracking
     */
    public boolean isQnavTracking() {
        return qnavTracking;
    }

    /**
     * @return count of times QuestNav tracking was lost since last restart
     */
    public int getQnavLostTrackingCount() {
        return qnavLostTrackingCount;
    }

    /**
     * @return current Quest battery percent
     */
    public int getQnavBatteryPercent() {
        return qnavBatteryPercent;
    }

    /**
     * @return current QuestNav connection latency in milliseconds
     */
    public double getQnavLatency() {
        return qnavLatency;
    }

    /**
     * @return the current estimated Pose3d from QuestNav WITH robot transform applied
     */
    public Pose3d getQnavRobotPose() {
        return qnavRobotPose;
    }

    /**
     * Set the Quest-reported ROBOT pose. Offset applied automatically. Where do you want the robot to think it is?
     *
     * @param pose the Pose3d to set to
     */
    public void setQnavRobotPose(Pose3d pose) {
        setQnavRawPose(pose.transformBy(ROBOT_TO_QUEST));
    }

    /**
     * Pose2d version of this method. All other values set to zero. Check whether you need 3d positioning data.
     *
     * @param pose the Pose2d to set to
     */
    public void setQnavRobotPose(Pose2d pose) {
        setQnavRobotPose(new Pose3d(pose));
    }

    /**
     * @return the current estimated Pose3d from QuestNav WITHOUT robot transform applied
     */
    public Pose3d getQnavRawPose() {
        return qnavRawPose;
    }

    /**
     * Set the raw Quest pose, with NO robot offsets included.
     *
     * @param pose the Pose3d to set to
     */
    public void setQnavRawPose(Pose3d pose) {
        questNav.setPose(pose);
    }

    /**
     * @return whether measurements from the Quest are currently being used
     */
    public boolean getQnavUsingMeasurements() {
        return qnavUseMeasurements;
    }

    /**
     * @return how many times QuestNav has disagreed with AprilTags
     */
    public double getQnavFaultCounter() {
        return qnavFaultCounter;
    }

    /**
     * @return whether faults are less than the declared threshold
     */
    public boolean getQnavHealthy() {
        return qnavHealthy;
    }

    /**
     * RETURNS NULL IF NO TAGS PRESENT!!!
     *
     * @return current estimated Pose3d from PhotonVision
     */
    public Pose3d getPvRobotPose1() {
        return pvRobotPose1;
    }

    public Pose3d getPvRobotPose2() {
        return pvRobotPose2;
    }

    public Pose3d getPvAvgRobotPose() {
        return pvAvgRobotPose;
    }

    /**
     * @return whether PhotonVision camera is connected
     */
    public boolean isPvConnected() {
        return pvConnected;
    }

    /**
     * @return whether or not PhotonVision has a target
     */
    public boolean getPvHasTarget() {
        return pvHasTarget;
    }

    /**
     * @return the ID of the best AprilTag target, -1 if no targets
     */
    public int getPvBestTargetId() {
        return pvBestTargetId;
    }

    /**
     * Set whether Quest measurements are being used for pose estimation
     */
    public void setQnavUseMeasurements(boolean b) {
        qnavUseMeasurements = b;
    }

    public Command enableQnavMeasurements() {
        return Commands.runOnce(() -> setQnavUseMeasurements(true))
            .ignoringDisable(true);
    }

    public Command disableQnavMeasurements() {
        return Commands.runOnce(() -> setQnavUseMeasurements(false))
            .ignoringDisable(true);
    }

    /**
     * Set whether PhotonVision measurements are being used for pose estimation
     */
    public void setPVUseMeasurements(boolean b) {
        pvUseMeasurements = b;
    }

    public Command enablePVMeasurements() {
        return Commands.runOnce(() -> setPVUseMeasurements(true))
            .ignoringDisable(true);
    }

    public Command disablePVMeasurements() {
        return Commands.runOnce(() -> setPVUseMeasurements(false))
            .ignoringDisable(true);
    }

    /**
     * Internal method which dynamically updates standard deviations based on number of tags
     *
     * @param estimatedPose estimated robot pose
     * @param targets       List of PhotonTrackedTargets
     */
    private Matrix<N3, N1> updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets,
        PhotonPoseEstimator poseEstimator
    ) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            return Constants.VisionConstants.PV_SINGLE_TAG_STD_DEVS;
        } else {
            // Pose present. Start running heuristic.
            var estStdDevs = Constants.VisionConstants.PV_SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we count, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

                if (tagPose.isEmpty()) continue;

                numTags++;

                avgDist +=
                    tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(
                            estimatedPose.get()
                                .estimatedPose
                                .toPose2d()
                                .getTranslation()
                        );
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                return Constants.VisionConstants.PV_SINGLE_TAG_STD_DEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;

                // Increase std devs if multiple targets are visible.
                if (numTags > 1) {
                    estStdDevs = Constants.VisionConstants.PV_MULTI_TAG_STD_DEVS;
                }

                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4) {
                    estStdDevs = VecBuilder.fill(
                        Double.MAX_VALUE,
                        Double.MAX_VALUE,
                        Double.MAX_VALUE
                    );
                } else {
                    estStdDevs =
                        estStdDevs.times(1 + (avgDist * avgDist / 30));
                }

                return estStdDevs;
            }
        }
    }

    /**
     * Gets a weight for a camera based on AprilTag ambiguity.
     *
     * Lower ambiguity means a more trustworthy pose.
     *
     * @param result PhotonVision pipeline result
     * @return camera weight
     */
    private double getAmbiguityWeight(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return 0.0;
        }

        double totalAmbiguity = 0.0;
        int count = 0;

        for (PhotonTrackedTarget target : result.getTargets()) {
            double ambiguity = target.getPoseAmbiguity();

            if (ambiguity >= 0) {
                totalAmbiguity += ambiguity;
                count++;
            }
        }

        // If PhotonVision does not provide ambiguity information,
        // give the camera a neutral weight.
        if (count == 0) {
            return 1.0;
        }

        double averageAmbiguity = totalAmbiguity / count;

        // Lower ambiguity = higher weight
        return 1.0 / (averageAmbiguity + 0.01);
    }

    /**
     * Combines two PhotonVision poses using ambiguity-based weights.
     *
     * @param pose1 first camera pose
     * @param weight1 first camera weight
     * @param pose2 second camera pose
     * @param weight2 second camera weight
     * @return weighted combined pose
     */
    private Pose3d combinePoses(
        Pose3d pose1,
        double weight1,
        Pose3d pose2,
        double weight2
    ) {
        double totalWeight = weight1 + weight2;

        // This should never happen because this method is only called
        // when both cameras have valid pose estimates.
        if (totalWeight <= 0) {
            return pose1;
        }

        double x =
            (pose1.getX() * weight1 + pose2.getX() * weight2)
                / totalWeight;

        double y =
            (pose1.getY() * weight1 + pose2.getY() * weight2)
                / totalWeight;

        double z =
            (pose1.getZ() * weight1 + pose2.getZ() * weight2)
                / totalWeight;

        double rotation1 = pose1.getRotation().getZ();
        double rotation2 = pose2.getRotation().getZ();

        //better to just use the one with the largest weight, or just the first camera if both are the same
        double rotation = 0;

        if (weight2 > weight1) {
            rotation = rotation2;
        } else {
            rotation = rotation1;
        }

        return new Pose3d(
            x,
            y,
            z,
            new Rotation3d(0, 0, rotation)
        );
    }

    /**
     * Sets QuestNav pose to PhotonVision pose, if tags are viewable
     */
    public Command syncPoses() {
        return Commands.runOnce(() -> {
            Pose3d p = getPvAvgRobotPose();

            if (p != null && pvHasTarget && pvConnected) {
                setQnavRobotPose(p);
                qnavFaultCounter = 0;

                System.out.println(
                    "[i] QuestNav pose reset successfully"
                );
            } else {
                System.out.println(
                    "[!] QuestNav pose reset failed: no targets or not connected"
                );
            }
        }).ignoringDisable(true);
    }

    /**
     * @param pose the Pose3d to check
     * @return whether or not the pose is within field dimensions
     */
    private boolean validFieldPose(Pose3d pose) {
        return pose.getX() < 0.0
            || pose.getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength()
            || pose.getY() < 0.0
            || pose.getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth();
    }

    /**
     * Handles periodic updates for QuestNav pose estimation.
     * <p>
     * Processes unread QuestNav pose frames, validates poses against AprilTag estimates,
     * manages fault detection, and adds valid vision measurements.
     *
     * @param bestEstimate the best AprilTag pose estimate for comparison, or null if none available
     */
    public void qnavPeriodic(Pose3d bestEstimate) {
        questNav.commandPeriodic(); // required by the headset

        // Update everything but poses first
        qnavConnected = questNav.isConnected();

        qnavTracking = questNav.isTracking();

        OptionalInt optionalInt = questNav.getTrackingLostCounter();
        qnavLostTrackingCount =
            optionalInt.isPresent() ? optionalInt.getAsInt() : -1;

        optionalInt = questNav.getBatteryPercent();
        qnavBatteryPercent =
            optionalInt.isPresent() ? optionalInt.getAsInt() : -1;

        qnavLatency = questNav.getLatency();

        boolean isQuestWorking = qnavConnected && qnavTracking;

        if (!isQuestWorking &&
            qnavFaultCounter < QUESTNAV_FAILURE_THRESHOLD) {
            qnavFaultCounter++;
        }

        // Iterate backwards through frames to find the most recent valid frame
        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

        for (int i = frames.length - 1; i >= 0; i--) {
            PoseFrame frame = frames[i];

            if (frame.isTracking()) {
                qnavRawPose = frame.questPose3d();

                qnavRobotPose =
                    qnavRawPose.transformBy(ROBOT_TO_QUEST.inverse());

                double bestEstimateDistance =
                    (bestEstimate != null && pvHasTarget)
                        ? qnavRobotPose
                            .toPose2d()
                            .getTranslation()
                            .getDistance(
                                bestEstimate
                                    .toPose2d()
                                    .getTranslation()
                            )
                        : 0;

                if (
                    bestEstimateDistance >
                    QUESTNAV_APRILTAG_ERROR_THRESHOLD.in(Meters)
                ) {
                    Logger.recordOutput(
                        "BestEstimateDistance",
                        bestEstimateDistance
                    );

                    // QuestNav disagrees with AprilTag vision:
                    // increment fault counter
                    if (
                        qnavFaultCounter <
                        QUESTNAV_FAILURE_THRESHOLD
                    ) {
                        qnavFaultCounter +=
                            Math.pow(bestEstimateDistance, 2);
                    }
                } else if (bestEstimate != null) {
                    // QuestNav agrees with AprilTags -
                    // decrement counter to allow recovery
                    qnavFaultCounter =
                        Math.max(qnavFaultCounter - 1.0, 0.0);
                }

                // Healthy if we're under the threshold or PV not working
                qnavHealthy =
                    qnavFaultCounter < QUESTNAV_FAILURE_THRESHOLD;

                // Only use QuestNav measurements when the switch is on
                if (qnavUseMeasurements && qnavHealthy) {
                    // Check whether or not the pose is within field bounds
                    Drivetrain.getInstance().addVisionMeasurement(
                        qnavRobotPose.toPose2d(),
                        frame.dataTimestamp(),
                        QNAV_STD_DEVS
                    );
                }

                break; // found the most recent tracking frame, exit loop
            }
        }

        // Log all values
        Logger.recordOutput(
            "LocalizationSystem/QNav/Connected",
            isQnavConnected()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/Tracking",
            isQnavTracking()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/LostTrackingCount",
            getQnavLostTrackingCount()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/BatteryPercent",
            getQnavBatteryPercent()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/Latency",
            getQnavLatency()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/RobotPose",
            getQnavRobotPose()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/RawPose",
            getQnavRawPose()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/UsingMeasurements",
            getQnavUsingMeasurements()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/FaultCounter",
            getQnavFaultCounter()
        );

        Logger.recordOutput(
            "LocalizationSystem/QNav/Healthy",
            getQnavHealthy()
        );
    }

    public void pvPeriodic() {

        // Update connection status
        pvConnected =
            pvCam.isConnected() || pvCam2.isConnected();

        // Grab all unread results from both cameras
        List<PhotonPipelineResult> pvCamResults =
            pvCam.getAllUnreadResults();

        List<PhotonPipelineResult> pvCam2Results =
            pvCam2.getAllUnreadResults();

        Optional<EstimatedRobotPose> visionEst1 =
            Optional.empty();

        Optional<EstimatedRobotPose> visionEst2 =
            Optional.empty();

        PhotonPipelineResult pvCamResult = null;
        PhotonPipelineResult pvCam2Result = null;

        // Process camera 1
        if (!pvCamResults.isEmpty()) {
            pvCamResult =
                pvCamResults.get(pvCamResults.size() - 1);

            // Attempt multi-tag estimation
            visionEst1 =
                pvEstimator1.estimateCoprocMultiTagPose(
                    pvCamResult
                );

            // If multi-tag fails, fall back to lowest ambiguity
            if (visionEst1.isEmpty()) {
                visionEst1 =
                    pvEstimator1.estimateLowestAmbiguityPose(
                        pvCamResult
                    );
            }
        }

        // Process camera 2
        if (!pvCam2Results.isEmpty()) {
            pvCam2Result =
                pvCam2Results.get(pvCam2Results.size() - 1);

            // Attempt multi-tag estimation
            visionEst2 =
                pvEstimator2.estimateCoprocMultiTagPose(
                    pvCam2Result
                );

            // If multi-tag fails, fall back to lowest ambiguity
            if (visionEst2.isEmpty()) {
                visionEst2 =
                    pvEstimator2.estimateLowestAmbiguityPose(
                        pvCam2Result
                    );
            }
        }

        // Determine whether either camera sees a target
        pvHasTarget =
            (pvCamResult != null && pvCamResult.hasTargets()) ||
            (pvCam2Result != null && pvCam2Result.hasTargets());

        // Determine best target ID
        if (
            pvCamResult != null &&
            pvCamResult.hasTargets()
        ) {
            pvBestTargetId =
                pvCamResult.getBestTarget().getFiducialId();
        } else if (
            pvCam2Result != null &&
            pvCam2Result.hasTargets()
        ) {
            pvBestTargetId =
                pvCam2Result.getBestTarget().getFiducialId();
        } else {
            pvBestTargetId = -1;
        }

        // Calculate ambiguity-based weights
        pvWeight =
            pvCamResult != null
                ? getAmbiguityWeight(pvCamResult)
                : 0.0;

        pvWeight2 =
            pvCam2Result != null
                ? getAmbiguityWeight(pvCam2Result)
                : 0.0;

        // A camera without a valid pose should not contribute
        if (visionEst1.isEmpty()) {
            pvWeight = 0.0;
        }

        if (visionEst2.isEmpty()) {
            pvWeight2 = 0.0;
        }

        // Update standard deviations using whichever
        // camera produced a valid estimate.
        if (
            visionEst1.isPresent() &&
            pvCamResult != null
        ) {
            pvStdDevs1 = updateEstimationStdDevs(
                visionEst1,
                pvCamResult.getTargets(),
                pvEstimator1
            );
        }
        
        if (
            visionEst2.isPresent() &&
            pvCam2Result != null
        ) {
            pvStdDevs2 = updateEstimationStdDevs(
                visionEst2,
                pvCam2Result.getTargets(),
                pvEstimator2
            );
        }

        /*
         * Determine the PhotonVision pose.
         *
         * Both cameras:
         *     Combine them using ambiguity weights.
         *
         * Camera 1 only:
         *     Use Camera 1.
         *
         * Camera 2 only:
         *     Use Camera 2.
         *
         * Neither camera:
         *     Do nothing. The previous valid pose is retained.
         */
        if (
            visionEst1.isPresent() &&
            visionEst2.isPresent()
        ) {
            pvRobotPose1 = visionEst1.get().estimatedPose;
            pvRobotPose2 = visionEst2.get().estimatedPose;
            pvAvgRobotPose =
                combinePoses(
                    visionEst1.get().estimatedPose,
                    pvWeight,
                    visionEst2.get().estimatedPose,
                    pvWeight2
                );
        } else if (visionEst1.isPresent()) {
            pvRobotPose1 = visionEst1.get().estimatedPose;
            pvAvgRobotPose = pvRobotPose1;
        } else if (visionEst2.isPresent()) {
            pvRobotPose2 = visionEst2.get().estimatedPose;
            pvAvgRobotPose = pvRobotPose2;
        }

        /*
         * Add PhotonVision measurement to drivetrain.
         *
         * IMPORTANT:
         * If neither camera sees a tag, neither visionEst is present,
         * so this block does nothing.
         */
        if (
            pvUseMeasurements &&
            (visionEst1.isPresent() || visionEst2.isPresent())
        ) {

            //if not using quest measurements, use photon vision measurements
            //from either camera, or both if both are available
            if (visionEst1.isPresent()) {
                double timestamp = visionEst1.get().timestampSeconds;
                if (!qnavHealthy || !qnavUseMeasurements) {
                    Drivetrain.getInstance().addVisionMeasurement(
                        pvRobotPose1.toPose2d(),
                        timestamp,
                        pvStdDevs1);
                }
            }

            if (visionEst2.isPresent()) {
                double timestamp = visionEst2.get().timestampSeconds;
                if (!qnavHealthy || !qnavUseMeasurements) {
                    Drivetrain.getInstance().addVisionMeasurement(
                        pvRobotPose2.toPose2d(), 
                        timestamp, 
                        pvStdDevs2);
                }
            }
        }

        // Log all values
        Logger.recordOutput(
            "LocalizationSystem/PV/Connected",
            pvConnected
        );

        Logger.recordOutput(
            "LocalizationSystem/PV/UsingMeasurements",
            pvUseMeasurements
        );

        Logger.recordOutput(
            "LocalizationSystem/PV/HasTargets",
            pvHasTarget
        );

        Logger.recordOutput(
            "LocalizationSystem/PV/BestTargetID",
            pvBestTargetId
        );

        Logger.recordOutput(
            "LocalizationSystem/PV/RobotPose1",
            pvRobotPose1
        );

        Logger.recordOutput(
            "LocalizationSystem/PV/RobotPose2",
            pvRobotPose2
        );

        Logger.recordOutput(
            "LocalizationSystem/PV/AverageRobotPose",
            pvAvgRobotPose
        );

        Logger.recordOutput(
            "LocalizationSystem/PV/Camera1Weight",
            pvWeight
        );

        Logger.recordOutput(
            "LocalizationSystem/PV/Camera2Weight",
            pvWeight2
        );
    }

    @Override
    public void periodic() {

        // PhotonVision periodic
        pvPeriodic();

        // QuestNav periodic
        qnavPeriodic(pvAvgRobotPose);
    }
}