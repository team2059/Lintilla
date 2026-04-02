package org.team2059.Lintilla.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

/**
 * Command to drive the robot using a flight stick controller.
 * Contains various methods of navigation including hub-tracking.
 */
public class TeleopDriveCommand extends Command {
	private final Drivetrain drivetrain;

	private final DoubleSupplier forwardX, forwardY, rotation, slider;
	private final BooleanSupplier strafeOnly, inverted, hubTracking, snakeMode;
	private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

	// Values for autorotation to hub
	private final PIDController controller;
	private final LoggedTunableNumber kP = new LoggedTunableNumber("HubTurnKp", 8.0);
	private final LoggedTunableNumber kI = new LoggedTunableNumber("HubTurnKi", 0);
	private final LoggedTunableNumber kD = new LoggedTunableNumber("HubTurnKd", 0);
	private final Translation2d shooterOffset = Constants.VisionConstants.SHOOTER_OFFSET.getTranslation();

	/**
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
	  DoubleSupplier forwardX,
	  DoubleSupplier forwardY,
	  DoubleSupplier rotation,
	  DoubleSupplier slider,
	  BooleanSupplier strafeOnly,
	  BooleanSupplier inverted,
	  BooleanSupplier hubTracking,
	  BooleanSupplier snakeMode
	) {

		this.drivetrain = drivetrain;
		this.forwardX = forwardX;
		this.forwardY = forwardY;
		this.rotation = rotation;
		this.slider = slider;
		this.strafeOnly = strafeOnly;
		this.inverted = inverted;
		this.hubTracking = hubTracking;
		this.snakeMode = snakeMode;

		this.xLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION);
		this.yLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ACCELERATION);
		this.rotLimiter = new SlewRateLimiter(DrivetrainConstants.MAX_ANGULAR_ACCELERATION);

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

		// Check for updated TunableNumbers for rotation controller and take action if needed
		LoggedTunableNumber.ifChanged(
		  hashCode(),
		  () -> {
			  controller.setPID(kP.get(), kI.get(), kD.get());
		  },
		  kP, kI, kD
		);

		// Get joystick input as x, y, and rotation
		double xSpeed = -forwardX.getAsDouble();
		double ySpeed = -forwardY.getAsDouble();
		double rot = -rotation.getAsDouble();


		// Apply deadband
		xSpeed = Math.abs(xSpeed) > 0.25 ? xSpeed : 0.0;
		ySpeed = Math.abs(ySpeed) > 0.25 ? ySpeed : 0.0;
		rot = Math.abs(rot) > 0.15 ? rot : 0.0;


		// Make the driving smoother
		xSpeed = xLimiter.calculate(xSpeed) * DrivetrainConstants.TELE_DRIVE_MAX_SPEED;
		ySpeed = yLimiter.calculate(ySpeed) * DrivetrainConstants.TELE_DRIVE_MAX_SPEED;
		rot = rotLimiter.calculate(rot) * DrivetrainConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

		// Apply slider limit
		double sliderVal = (-slider.getAsDouble() + 1) / 2;
		sliderVal = Math.max(sliderVal, 0.15);
		xSpeed *= sliderVal;
		ySpeed *= sliderVal;
		rot *= sliderVal;

		xSpeed = -MathUtil.applyDeadband(xSpeed, 0.1, 1);
		ySpeed = -MathUtil.applyDeadband(ySpeed, 0.1, 1);
		rot = -MathUtil.applyDeadband(rot, 0.3, 0.75);

		if (hubTracking.getAsBoolean()) {

			ShooterBase.getInstance()
			  .calculateSOTF(
				Drivetrain.getInstance().getEstimatedPose(),
			    Drivetrain.getInstance().getFieldRelativeSpeeds()
			  );

			// Apply PID to rotation
			double angularSpeedRps = controller.calculate(
			  drivetrain.getEstimatedPose().getRotation().getRadians(),
			  ShooterBase.getInstance().targetAimAngleRad
			);
			ShooterBase.getInstance().isAimed = controller.atSetpoint(); // set for use in other commands

			// Apply drive command
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
		} else if (snakeMode.getAsBoolean()) {
			Rotation2d snakeAngle = new Rotation2d(xSpeed, ySpeed);
			double snakeRotSpeed = 0;
			if (Math.abs(Math.hypot(xSpeed, ySpeed)) > 0.05) {
				Pose2d currentPose = drivetrain.getEstimatedPose();
				snakeRotSpeed = MathUtil.clamp(controller.calculate(currentPose.getRotation().getRadians(), snakeAngle.getRadians() - (-Math.PI / 2)), -1, 1);
				snakeRotSpeed *= DrivetrainConstants.MAX_ANGULAR_VELOCITY;

				drivetrain.drive(
				  xSpeed,
				  ySpeed,
				  -snakeRotSpeed, // TODO: Test this
				  Drivetrain.isFieldRelativeTeleop); // Should always be true for snake mode
			} else {
				drivetrain.drive(
				  xSpeed,
				  ySpeed,
				  rot,
				  Drivetrain.isFieldRelativeTeleop);
			}
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
