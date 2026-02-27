package org.team2059.Lintilla.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.Constants.AutoConstants;
import org.team2059.Lintilla.Constants.DrivetrainConstants;
import org.team2059.Lintilla.Constants.VisionConstants;
import org.team2059.Lintilla.routines.DrivetrainRoutine;

import java.util.Optional;

public class Drivetrain extends SubsystemBase {

	public static boolean isFieldRelativeTeleop = true;

	public final DrivetrainRoutine routine;

	private final GyroscopeIO gyroIO;
	private final GyroscopeIOInputsAutoLogged gyroInputs = new GyroscopeIOInputsAutoLogged();

	private final SwerveModuleIO[] modules;
	private final SwerveModuleIOInputsAutoLogged[] swerveModuleInputs = new SwerveModuleIOInputsAutoLogged[4];

	private final Field2d field = new Field2d();

	private final SwerveDrivePoseEstimator poseEstimator;

	public Drivetrain(
	  GyroscopeIO gyroIO,
	  SwerveModuleIO frontLeftModuleIO,
	  SwerveModuleIO frontRightModuleIO,
	  SwerveModuleIO backLeftModuleIO,
	  SwerveModuleIO backRightModuleIO
	) {

		this.gyroIO = gyroIO;

		modules = new SwerveModuleIO[4];
		modules[0] = frontLeftModuleIO;
		modules[1] = frontRightModuleIO;
		modules[2] = backLeftModuleIO;
		modules[3] = backRightModuleIO;

		for (int i = 0; i < 4; i++) {
			swerveModuleInputs[i] = new SwerveModuleIOInputsAutoLogged();
		}

		// Reset encoders so we start tracking distances from zero
		frontLeftModuleIO.resetEncoders();
		frontRightModuleIO.resetEncoders();
		backLeftModuleIO.resetEncoders();
		backRightModuleIO.resetEncoders();

		// Estimates our pose on the field using vision system, if available.
		// Behaves just like SwerveDriveOdometry, just with optional
		// vision measurements.
		poseEstimator = new SwerveDrivePoseEstimator(
		  DrivetrainConstants.kinematics,
		  getHeading(),
		  getModulePositions(),
		  new Pose2d()
		);

		routine = new DrivetrainRoutine(this);

		// Configure AutoBuilder last
		configureAutoBuilder();

		PathPlannerLogging.setLogTargetPoseCallback((pose) -> { // target pose
			field.getObject("target pose").setPose(pose);
		});

		PathPlannerLogging.setLogActivePathCallback((poses) -> { // active path (list of poses)
			field.getObject("trajectory").setPoses(poses);
		});

		SmartDashboard.putData(field);
	}

	/**
	 * @return current gyroscope heading
	 */
	public Rotation2d getHeading() {
		return gyroIO.getRotation2d();
	}

	/**
	 * @return current pose from pose estimator
	 */
	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * @return current swerve module states of all modules
	 */
	public SwerveModuleState[] getStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getState();
		}

		return states;
	}

	/**
	 * @return current module positions array
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (int i = 0; i < 4; i++) {
			positions[i] = new SwerveModulePosition(
			  swerveModuleInputs[i].drivePos,
			  new Rotation2d(swerveModuleInputs[i].azimuthAbsolutePos)
			);
		}

		return positions;
	}

	public SwerveModuleIO getFrontLeft() {
		return modules[0];
	}

	public SwerveModuleIO getFrontRight() {
		return modules[1];
	}

	public SwerveModuleIO getBackLeft() {
		return modules[2];
	}

	public SwerveModuleIO getBackRight() {
		return modules[3];
	}

	public SwerveModuleIOInputsAutoLogged[] getSwerveInputs() {
		return swerveModuleInputs;
	}

	/**
	 * @return ChassisSpeeds of current robot-relative speeds
	 */
	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DrivetrainConstants.kinematics.toChassisSpeeds(getStates());
	}

	/**
	 * @return current Pose2d of the shooter. The offset has been applied. Where is the shooter relative to the field?
	 */
	public Pose2d getShooterPose() {
		return getEstimatedPose().transformBy(VisionConstants.SHOOTER_OFFSET);
	}

	public void resetGyroHeading() {
		gyroIO.reset();
	}

	public void setFieldRelativity() {
		if (isFieldRelativeTeleop) {
			isFieldRelativeTeleop = false;
		} else {
			isFieldRelativeTeleop = true;
		}
	}

	/**
	 * Method to set module states
	 *
	 * @param desiredStates SwerveModuleState[] desired states
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		// makes it never go above specified max velocity
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxVelocity);

		// Sets the speed and rotation of each module
		for (int i = 0; i < 4; i++) {
			modules[i].setState(desiredStates[i]);
		}
	}

	/**
	 * Resets position based on given pose, offsetting the gyroscope
	 *
	 * @param pose Pose2d to reset to
	 */
	public void setPosition(Pose2d pose) {
		poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
	}

	/**
	 * Method to drive the robot either field or robot relative
	 *
	 * @param forward         forward/backward linear velocity component (meters/sec)
	 * @param strafe          strafe linear velocity component (meters/sec)
	 * @param rotation        rotational component (radians/sec)
	 * @param isFieldRelative should drive field relative or not
	 */
	public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

		/*
		 * ChassisSpeeds object to represent the overall state of the robot
		 * ChassisSpeeds takes a forward and sideways linear value and a rotational
		 * value
		 *
		 * speeds is set to field relative or default (robot relative) based on
		 * parameter
		 */

		ChassisSpeeds speeds = isFieldRelative
		  ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
		  : new ChassisSpeeds(forward, strafe, rotation);

		speeds = ChassisSpeeds.discretize(speeds, 0.02);

		Logger.recordOutput("RotationRate", speeds.omegaRadiansPerSecond);

		// use kinematics (wheel placements) to convert overall robot state to array of
		// individual module states
		SwerveModuleState[] states = DrivetrainConstants.kinematics.toSwerveModuleStates(speeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);

		setModuleStates(states);

	}

	/**
	 * Method to drive robot-relative
	 *
	 * @param chassisSpeeds desired speeds
	 */
	public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

		SwerveModuleState[] newStates = DrivetrainConstants.kinematics.toSwerveModuleStates(discreteSpeeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DrivetrainConstants.maxVelocity);

		setModuleStates(newStates);
	}

	/**
	 * Method to drive field-relative
	 *
	 * @param chassisSpeeds desired speeds
	 */
	public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

		chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getHeading());

		SwerveModuleState[] newStates = DrivetrainConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DrivetrainConstants.maxVelocity);

		setModuleStates(newStates);
	}

	/**
	 * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
	 * while still accounting for measurement noise.
	 *
	 * @param visionPoseMeters         The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds         The timestamp of the vision measurement in seconds.
	 * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
	 *                                 in the form [x, y, theta]ᵀ, with units in meters and radians.
	 */
	public void addVisionMeasurement(
	  Pose2d visionPoseMeters,
	  double timestampSeconds,
	  Matrix<N3, N1> visionMeasurementStdDevs
	) {
		poseEstimator.addVisionMeasurement(visionPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

	/**
	 * AutoBuilder configuration method, must be run after all other configuration
	 */
	public void configureAutoBuilder() {

		System.out.println("Configuring Auto Builder...");

		try {
			RobotConfig config = RobotConfig.fromGUISettings();

			// Configure AutoBuilder
			AutoBuilder.configure(
			  this::getEstimatedPose, // Robot pose supplier
			  this::setPosition, // Method to reset pose. We use the Quest method here.
			  this::getRobotRelativeSpeeds, // ChassisSpeeds supplier, MUST be robot relative
			  (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given robot-relative chassis speeds
			  new PPHolonomicDriveController(
				new PIDConstants(AutoConstants.kAutoTranslationP, 0.0, AutoConstants.kAutoTranslationD),
				new PIDConstants(AutoConstants.kAutoRotationP, 0.0, AutoConstants.kAutoRotationD)),
			  config,
			  () -> {
				  // Boolean supplier that controls when the path will be mirrored for the red
				  // alliance
				  // This will flip the path being followed to the red side of the field
				  // The origin will remain on the blue side
				  var alliance = DriverStation.getAlliance();
				  if (alliance.isPresent()) {
					  return alliance.get() == DriverStation.Alliance.Red;
				  }
				  return false;
			  },
			  this // reference to this subsystem to set requirements
			);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Calculate the direct horizontal distance from the shooter to the Alliance Hub.
	 *
	 * @return distance, in METERS
	 */
	public double calculateDistanceShooterToHubMeters() {
		// If we can't do anything, return a negative distance (that's not valid)
		double output = -1;

		// Checks for alliance and selects appropriate hub pose
		Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent()) {
			if (ally.get() == DriverStation.Alliance.Red) {
				output = VisionConstants.RED_HUB_CENTER.getDistance(getShooterPose().getTranslation());
			}
			if (ally.get() == DriverStation.Alliance.Blue) {
				output = VisionConstants.BLUE_HUB_CENTER.getDistance(getShooterPose().getTranslation());
			}
		} else {
			System.out.println("ALLY NOT AVAILABLE");
		}

		return output;
	}

	@Override
	public void periodic() {

		Logger.recordOutput("Estimated Pose", getEstimatedPose());
		Logger.recordOutput("Shooter Pose", getShooterPose());
		Logger.recordOutput("Field Relative", isFieldRelativeTeleop);

		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Gyroscope", gyroInputs);

		for (int i = 0; i < 4; i++) {
			modules[i].updateInputs(swerveModuleInputs[i]);
			Logger.processInputs(("Drive/Module" + Integer.toString(i)), swerveModuleInputs[i]);
		}

		// Update pose estimator based on wheel positions
		poseEstimator.update(getHeading(), getModulePositions());

		field.setRobotPose(getEstimatedPose());
	}
}