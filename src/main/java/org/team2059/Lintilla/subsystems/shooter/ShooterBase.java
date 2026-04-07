package org.team2059.Lintilla.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.RobotContainer;

import static edu.wpi.first.units.Units.*;
import static org.team2059.Lintilla.Constants.OperatorConstants.*;
import static org.team2059.Lintilla.Constants.ShooterConstants;
import static org.team2059.Lintilla.Constants.VisionConstants.SHOOTER_OFFSET;
import static org.team2059.Lintilla.Constants.VisionConstants.getHubTranslation;

public class ShooterBase extends SubsystemBase {

	private static ShooterBase instance;

	public final ShooterIO leftShooter;
	public final ShooterIO rightShooter;

	public final ShooterIOInputsAutoLogged leftShooterInputs = new ShooterIOInputsAutoLogged();
	public final ShooterIOInputsAutoLogged rightShooterInputs = new ShooterIOInputsAutoLogged();

	private final SysIdRoutine leftFlywheelRoutine;
	private final SysIdRoutine leftIndexerRoutine;
	private final SysIdRoutine rightFlywheelRoutine;
	private final SysIdRoutine rightIndexerRoutine;
	private final MutVoltage appliedVoltsRoutine;
	private final MutAngle angleRoutine;
	private final MutAngularVelocity angularVelocityRoutine;

	public boolean isAimed = false;

	public boolean addFivePercent;
	public boolean subFivePercent;

	public double currentDistanceToTarget = 0.0;

	public double desiredRPM = 0.0;

	public double targetAimAngleRad = 0.0;

	private ShooterBase(
	  ShooterIO leftShooter,
	  ShooterIO rightShooter
	) {
		this.leftShooter = leftShooter;
		this.rightShooter = rightShooter;

		addFivePercent = !RobotContainer.buttonBox.getRawButton(SHOOTER_ADD5PERCENT_SWITCH);
		subFivePercent = !RobotContainer.buttonBox.getRawButton(SHOOTER_SUB5PERCENT_SWITCH);

		if (tuningMode) {
			// Declare SysID routines
			appliedVoltsRoutine = Volts.mutable(0);
			angleRoutine = Rotations.mutable(0);
			angularVelocityRoutine = RPM.mutable(0);
			leftFlywheelRoutine = new SysIdRoutine(
			  new SysIdRoutine.Config(),
			  new SysIdRoutine.Mechanism(
				voltage -> {
					leftShooter.setFlywheelVoltage(voltage.in(Volts));
				},
				log -> {
					log.motor("left-shooter-motor")
					  .voltage(appliedVoltsRoutine.mut_replace(leftShooterInputs.flywheelAppliedVolts))
					  .angularPosition(angleRoutine.mut_replace(leftShooterInputs.flywheelPosition))
					  .angularVelocity(angularVelocityRoutine.mut_replace(leftShooterInputs.flywheelVelocity));
				},
				this
			  )
			);
			rightFlywheelRoutine = new SysIdRoutine(
			  new SysIdRoutine.Config(),
			  new SysIdRoutine.Mechanism(
				voltage -> {
					rightShooter.setFlywheelVoltage(voltage.in(Volts));
				},
				log -> {
					log.motor("right-shooter-motor")
					  .voltage(appliedVoltsRoutine.mut_replace(rightShooterInputs.flywheelAppliedVolts))
					  .angularPosition(angleRoutine.mut_replace(rightShooterInputs.flywheelPosition))
					  .angularVelocity(angularVelocityRoutine.mut_replace(rightShooterInputs.flywheelVelocity));
				},
				this
			  )
			);
			leftIndexerRoutine = new SysIdRoutine(
			  new SysIdRoutine.Config(),
			  new SysIdRoutine.Mechanism(
				voltage -> {
					leftShooter.setIndexerVoltage(voltage.in(Volts));
				},
				log -> {
					log.motor("left-shooter-motor")
					  .voltage(appliedVoltsRoutine.mut_replace(leftShooterInputs.indexerAppliedVolts))
					  .angularPosition(angleRoutine.mut_replace(leftShooterInputs.indexerPosition))
					  .angularVelocity(angularVelocityRoutine.mut_replace(leftShooterInputs.indexerVelocity));
				},
				this
			  )
			);
			rightIndexerRoutine = new SysIdRoutine(
			  new SysIdRoutine.Config(),
			  new SysIdRoutine.Mechanism(
				voltage -> {
					rightShooter.setIndexerVoltage(voltage.in(Volts));
				},
				log -> {
					log.motor("right-shooter-motor")
					  .voltage(appliedVoltsRoutine.mut_replace(rightShooterInputs.indexerAppliedVolts))
					  .angularPosition(angleRoutine.mut_replace(rightShooterInputs.indexerPosition))
					  .angularVelocity(angularVelocityRoutine.mut_replace(rightShooterInputs.indexerVelocity));
				},
				this
			  )
			);
		} else {
			leftFlywheelRoutine = null;
			rightFlywheelRoutine = null;
			leftIndexerRoutine = null;
			rightIndexerRoutine = null;
			appliedVoltsRoutine = null;
			angleRoutine = null;
			angularVelocityRoutine = null;
		}
	}

	public static ShooterBase getInstance() {
		if (instance == null) {
			throw new RuntimeException("ShooterBase is not initialized! Call initialize() first");
		}

		return instance;
	}

	public static void initialize(
	  ShooterIO leftShooter,
	  ShooterIO rightShooter
	) {
		if (instance == null) {
			instance = new ShooterBase(
			  leftShooter,
			  rightShooter
			);
		}
	}

	/**
	 * This method is calculates the current distance to the Hub, as well as
	 * calculate the required heading angle.
	 *
	 * @param robotPose   Current field-relative robot pose
	 * @param fieldSpeeds Current field-relative ChassisSpeeds
	 */
	public void calculateSOTF(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {

		// Check if robot is actually moving
		boolean isMoving =
		  Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond) > 0.1
			|| Math.abs(fieldSpeeds.omegaRadiansPerSecond) > 0.1;

		Translation2d virtualTarget = getHubTranslation();

		if (!isMoving) {
			// ROBOT IS STATIONARY: Skip SOTF and do standard static aiming
			this.currentDistanceToTarget = robotPose.getTranslation().getDistance(virtualTarget);
			this.targetAimAngleRad = Math.atan2(
			  virtualTarget.getY() - robotPose.getTranslation().getY(),
			  virtualTarget.getX() - robotPose.getTranslation().getX()
			);
			return; // Exit method early
		}

		Translation2d vRobot = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
		Translation2d vTan = new Translation2d(
		  -fieldSpeeds.omegaRadiansPerSecond * SHOOTER_OFFSET.getY(),
		  fieldSpeeds.omegaRadiansPerSecond * SHOOTER_OFFSET.getX()
		);
		Translation2d effectiveVelocity = vRobot.plus(vTan);

		// 4 Iterations for convergence
		for (int i = 0; i < 4; i++) {
			double distance = robotPose.getTranslation().getDistance(virtualTarget);
			double tof = getToF(distance) + ShooterConstants.SYSTEM_LATENCY_SECONDS;
			Translation2d offset = effectiveVelocity.times(tof);
			virtualTarget = getHubTranslation().minus(offset);
			this.currentDistanceToTarget = distance;
		}

		this.targetAimAngleRad = Math.atan2(
		  virtualTarget.getY() - robotPose.getTranslation().getY(),
		  virtualTarget.getX() - robotPose.getTranslation().getX()
		);
	}

	// SysID getters
	public Command leftShooterQuasiForward() {
		if (leftFlywheelRoutine == null) return Commands.none();
		return leftFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command rightShooterQuasiForward() {
		if (rightFlywheelRoutine == null) return Commands.none();
		return rightFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command leftIndexerQuasiForward() {
		if (leftIndexerRoutine == null) return Commands.none();
		return leftIndexerRoutine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command rightIndexerQuasiForward() {
		if (rightIndexerRoutine == null) return Commands.none();
		return rightIndexerRoutine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command leftShooterQuasiReverse() {
		if (leftFlywheelRoutine == null) return Commands.none();
		return leftFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	public Command rightShooterQuasiReverse() {
		if (rightFlywheelRoutine == null) return Commands.none();
		return rightFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	public Command leftIndexerQuasiReverse() {
		if (leftIndexerRoutine == null) return Commands.none();
		return leftIndexerRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	public Command rightIndexerQuasiReverse() {
		if (rightIndexerRoutine == null) return Commands.none();
		return rightIndexerRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	public Command leftShooterDynamicForward() {
		if (leftFlywheelRoutine == null) return Commands.none();
		return leftFlywheelRoutine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command rightShooterDynamicForward() {
		if (rightFlywheelRoutine == null) return Commands.none();
		return rightFlywheelRoutine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command leftIndexerDynamicForward() {
		if (leftIndexerRoutine == null) return Commands.none();
		return leftIndexerRoutine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command rightIndexerDynamicForward() {
		if (rightIndexerRoutine == null) return Commands.none();
		return rightIndexerRoutine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command leftShooterDynamicReverse() {
		if (leftFlywheelRoutine == null) return Commands.none();
		return leftFlywheelRoutine.dynamic(SysIdRoutine.Direction.kReverse);
	}

	public Command rightShooterDynamicReverse() {
		if (rightFlywheelRoutine == null) return Commands.none();
		return rightFlywheelRoutine.dynamic(SysIdRoutine.Direction.kReverse);
	}

	public Command leftIndexerDynamicReverse() {
		if (leftIndexerRoutine == null) return Commands.none();
		return leftIndexerRoutine.dynamic(SysIdRoutine.Direction.kReverse);
	}

	public Command rightIndexerDynamicReverse() {
		if (rightIndexerRoutine == null) return Commands.none();
		return rightIndexerRoutine.dynamic(SysIdRoutine.Direction.kReverse);
	}

	public Command unjamShooters() {
		return Commands.startEnd(
		  () -> {
			  leftShooter.setIndexerSpeed(-1);
			  rightShooter.setIndexerSpeed(-1);
		  },
		  this::stopAllSubsystemMotors
		);
	}

	/**
	 * Fetch the needed RPM of the flywheel to shoot fuel a given distance
	 *
	 * @param distanceMeters horizontal distance to target, in meters
	 * @return RPM to set the shooter at
	 */
	public double getTargetRpm(double distanceMeters) {
		desiredRPM = ShooterConstants.SHOOTER_MAP.get(distanceMeters).rpm();

		return desiredRPM;
	}

	/**
	 * Fetch the estimated Time of Flight for shooting from a certain distance
	 *
	 * @param distanceMeters distance from shooter to hub in meters
	 * @return time in seconds
	 */
	public double getToF(double distanceMeters) {
		return ShooterConstants.SHOOTER_MAP.get(distanceMeters).timeOfFlight();
	}

	public void stopAllSubsystemMotors() {
		leftShooter.stopFlywheel();
		leftShooter.stopIndexer();
		rightShooter.stopFlywheel();
		rightShooter.stopIndexer();
	}

	public void setAddFivePercent(boolean b) {
		addFivePercent = b;
	}

	public void setSubFivePercent(boolean b) {
		subFivePercent = b;
	}

	@Override
	public void periodic() {
		leftShooter.updateInputs(leftShooterInputs);
		rightShooter.updateInputs(rightShooterInputs);

		Logger.processInputs("ShooterBase/Left", leftShooterInputs);
		Logger.processInputs("ShooterBase/Right", rightShooterInputs);

		Logger.recordOutput("AimedAtHub", isAimed);
		Logger.recordOutput("+5%", addFivePercent);
		Logger.recordOutput("-5%", subFivePercent);
		Logger.recordOutput("CurrentDistanceToHub", currentDistanceToTarget);
		Logger.recordOutput("TargetAngleToHub", targetAimAngleRad);
	}
}
