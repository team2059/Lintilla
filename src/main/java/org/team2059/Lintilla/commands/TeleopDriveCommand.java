package org.team2059.Lintilla.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.Constants;
import org.team2059.Lintilla.Constants.DrivetrainConstants;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;
import org.team2059.Lintilla.util.LoggedTunableNumber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends Command {
	private final Drivetrain drivetrain;
	private final ShooterBase shooterBase;

	private final DoubleSupplier forwardX, forwardY, rotation, slider;
	private final BooleanSupplier strafeOnly, inverted, hubTracking;
	private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

	// Values for autorotation to hub
	private final PIDController controller;
	private final LoggedTunableNumber kP = new LoggedTunableNumber("HubTurnKp", 8.0);
	private final LoggedTunableNumber kI = new LoggedTunableNumber("HubTurnKi", 0);
	private final LoggedTunableNumber kD = new LoggedTunableNumber("HubTurnKd", 0);

	/**
	 * Command to drive the robot using a flight stick controller.
	 * Contains various methods of navigation including hub-tracking.
	 *
	 * @param drivetrain  the Drivetrain subsystem to interface with
	 * @param forwardX    X-axis input supplier
	 * @param forwardY    Y-axis input supplier
	 * @param rotation    Omega-axis input supplier
	 * @param slider      Slider input supplier (it's also an axis)
	 * @param strafeOnly  Button to indicate strafing only
	 * @param inverted    Button to indicate inversion
	 * @param hubTracking Button to indicate tracking Hub
	 */
	public TeleopDriveCommand(
	  Drivetrain drivetrain,
	  ShooterBase shooterBase,
	  DoubleSupplier forwardX,
	  DoubleSupplier forwardY,
	  DoubleSupplier rotation,
	  DoubleSupplier slider,
	  BooleanSupplier strafeOnly,
	  BooleanSupplier inverted,
	  BooleanSupplier hubTracking
	) {

		this.drivetrain = drivetrain;
		this.shooterBase = shooterBase;

		this.forwardX = forwardX;
		this.forwardY = forwardY;
		this.rotation = rotation;
		this.slider = slider;
		this.strafeOnly = strafeOnly;
		this.inverted = inverted;
		this.hubTracking = hubTracking;

		this.xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
		this.yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
		this.rotLimiter = new SlewRateLimiter(DrivetrainConstants.maxAngularAcceleration);

		// Configure controller to handle angles
		controller = new PIDController(kP.get(), kI.get(), kD.get());
		controller.enableContinuousInput(-Math.PI, Math.PI);

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Set tolerance to 1 degree for rotational PID (applies if button is selected)
		controller.setTolerance(Math.toRadians(2));
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		// Check for updated TunableNumbers for autorotation controller and take action if needed
		LoggedTunableNumber.ifChanged(
		  hashCode(),
		  () -> {
			  controller.setPID(kP.get(), kI.get(), kD.get());
		  },
		  kP, kI, kD
		);

		/**
		 * Units are given in meters/sec and radians/sec
		 * Since joysticks give output from -1 to 1, we multiply outputs by the max speed
		 * Otherwise, the max speed would be 1 m/s and 1 rad/s
		 */

		// Get joystick input as x, y, and rotation
		double xSpeed = -forwardX.getAsDouble();
		double ySpeed = -forwardY.getAsDouble();
		double rot = -rotation.getAsDouble();

		// Apply deadband
		xSpeed = Math.abs(xSpeed) > 0.25 ? xSpeed : 0.0;
		ySpeed = Math.abs(ySpeed) > 0.35 ? ySpeed : 0.0;
		rot = Math.abs(rot) > 0.4 ? rot : 0.0;

		// Make the driving smoother
		xSpeed = xLimiter.calculate(xSpeed) * DrivetrainConstants.kTeleDriveMaxSpeed;
		ySpeed = yLimiter.calculate(ySpeed) * DrivetrainConstants.kTeleDriveMaxSpeed;
		rot = rotLimiter.calculate(rot) * DrivetrainConstants.kTeleDriveMaxAngularSpeed;

		// Apply slider limit
		double sliderVal = (-slider.getAsDouble() + 1) / 2;
		sliderVal = sliderVal < 0.15 ? 0.15 : sliderVal;
		xSpeed *= sliderVal;
		ySpeed *= sliderVal;
		rot *= sliderVal;

		xSpeed = -MathUtil.applyDeadband(xSpeed, 0.1, 1);
		ySpeed = -MathUtil.applyDeadband(ySpeed, 0.1, 1);
		rot = -MathUtil.applyDeadband(rot, 0.3, 0.75);

		if (hubTracking.getAsBoolean()) { // Hub autoalignment flag
			Pose2d currentPose = drivetrain.getEstimatedPose();
			Translation2d shooterTranslation = drivetrain.getShooterPose().getTranslation();
			Translation2d robotTranslation = currentPose.getTranslation();

			// Get field relative velocity vector
			ChassisSpeeds currentSpeeds = drivetrain.getFieldRelativeSpeeds();
			Translation2d vRobot = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

			Translation2d shooterOffset = Constants.VisionConstants.SHOOTER_OFFSET.getTranslation();
			Translation2d vTan = new Translation2d(
			  -currentSpeeds.omegaRadiansPerSecond * shooterOffset.getY(),
			  currentSpeeds.omegaRadiansPerSecond * shooterOffset.getX()
			);

			Translation2d effectiveShooterVelocity = vRobot.plus(vTan);

			// Circular dependency loop: convergence on the perfect ToF and distance
			Translation2d virtualTarget = Constants.VisionConstants.getHubTranslation();
			Translation2d predictedOffset = new Translation2d();
			for (int i = 0; i < 4; i++) {
				// Measure distance to current virtual target guess
				double predictedDistance = shooterTranslation.getDistance(virtualTarget);

				// Total time of flight taking into account mechanical/system latency
				double timeOfFlight = shooterBase.getToF(predictedDistance) + Constants.ShooterConstants.SYSTEM_LATENCY_SECONDS;

				// Calculate how far the ball will drift across the field
				predictedOffset = effectiveShooterVelocity.times(timeOfFlight);

				// Shift the aim point in the opposite direction of the drift
				virtualTarget = Constants.VisionConstants.getHubTranslation().minus(predictedOffset);

				shooterBase.currentDistanceToTarget = predictedDistance;
			}

			// Calculate the angle needed to face the new target
			double targetAngleRad = Math.atan2(
			  virtualTarget.getY() - robotTranslation.getY(),
			  virtualTarget.getX() - robotTranslation.getX()
			);

			double currentAngleRad = currentPose.getRotation().getRadians();

			// Apply PID to rotation
			double angularSpeedRps = controller.calculate(currentAngleRad, targetAngleRad);
			shooterBase.isAimed = controller.atSetpoint();

			drivetrain.drive(xSpeed, ySpeed, angularSpeedRps, Drivetrain.isFieldRelativeTeleop);

		} else if (inverted.getAsBoolean()) { // Invert all axes if requested
			drivetrain.drive(
			  -xSpeed,
			  ySpeed,
			  -rot,
			  Drivetrain.isFieldRelativeTeleop
			);
		} else if (strafeOnly.getAsBoolean()) { // Strafe only relative to robot
			drivetrain.drive(
			  0,
			  ySpeed,
			  0,
			  true
			);
		} else { // Drive normally
			drivetrain.drive(
			  xSpeed,
			  ySpeed,
			  rot,
			  Drivetrain.isFieldRelativeTeleop
			);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, true);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
