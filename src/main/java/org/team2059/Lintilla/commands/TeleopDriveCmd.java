package org.team2059.Lintilla.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.Constants;
import org.team2059.Lintilla.Constants.DrivetrainConstants;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.util.LoggedTunableNumber;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDriveCmd extends Command {
	private final Drivetrain drivetrain;
	private final DoubleSupplier forwardX, forwardY, rotation, slider;
	private final BooleanSupplier strafeOnly, inverted, autoAlignHub;
	private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

	// Values for autorotation to hub
	private final Translation2d hub;
	private final PIDController controller;
	private final LoggedTunableNumber kP = new LoggedTunableNumber("HubTurnKp", 5.0);
	private final LoggedTunableNumber kI = new LoggedTunableNumber("HubTurnKi", 0);
	private final LoggedTunableNumber kD = new LoggedTunableNumber("HubTurnKd", 0.1);

	/**
	 * Creates a new TeleopDriveCmd.
	 */
	public TeleopDriveCmd(
	  Drivetrain drivetrain,
	  DoubleSupplier forwardX,
	  DoubleSupplier forwardY,
	  DoubleSupplier rotation,
	  DoubleSupplier slider,
	  BooleanSupplier strafeOnly,
	  BooleanSupplier inverted,
	  BooleanSupplier autoAlignHub
	) {

		this.drivetrain = drivetrain;
		this.forwardX = forwardX;
		this.forwardY = forwardY;
		this.rotation = rotation;
		this.slider = slider;
		this.strafeOnly = strafeOnly;
		this.inverted = inverted;
		this.autoAlignHub = autoAlignHub;

		this.xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
		this.yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
		this.rotLimiter = new SlewRateLimiter(DrivetrainConstants.maxAngularAcceleration);

		// Set the appropriate hub constant. There should be a color received by the time this command is run.
		// However, blue is the default if there is no color yet.
		Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent()) {
			if (ally.get() == DriverStation.Alliance.Red) {
				hub = Constants.VisionConstants.RED_HUB_CENTER;
			} else if (ally.get() == DriverStation.Alliance.Blue) {
				hub = Constants.VisionConstants.BLUE_HUB_CENTER;
			} else {
				hub = Constants.VisionConstants.BLUE_HUB_CENTER;
			}
		} else {
			hub = Constants.VisionConstants.BLUE_HUB_CENTER;
		}

		// Configure controller to handle angles
		controller = new PIDController(kP.get(), kI.get(), kD.get());
		controller.enableContinuousInput(-Math.PI, Math.PI);

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		controller.setTolerance(Math.toRadians(1));
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

		if (autoAlignHub.getAsBoolean()) { // Hub autoalignment flag
			Pose2d currentPose = drivetrain.getEstimatedPose();

			double targetAngleRad = Math.atan2(hub.getY() - currentPose.getY(), hub.getX() - currentPose.getX());
			double currentAngleRad = currentPose.getRotation().getRadians();

			double angularSpeedRps = controller.calculate(currentAngleRad, targetAngleRad);

			drivetrain.drive(xSpeed, ySpeed, angularSpeedRps, drivetrain.isFieldRelativeTeleop);
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
		drivetrain.drive(0,0,0,true);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
