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
import static org.team2059.Lintilla.Constants.VisionConstants.getHubTranslation;

public class ShooterBase extends SubsystemBase {

	private static ShooterBase instance;

	public final ShooterIO shooter;

	public final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

	private final SysIdRoutine drumRoutine;
	private final SysIdRoutine indexerRoutine;
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
		ShooterIO shooter
	) {
		this.shooter = shooter;

		addFivePercent = !RobotContainer.buttonBox.getRawButton(SHOOTER_ADD5PERCENT_SWITCH);
		subFivePercent = !RobotContainer.buttonBox.getRawButton(SHOOTER_SUB5PERCENT_SWITCH);

		if (tuningMode) {
			// Declare SysID routines
			appliedVoltsRoutine = Volts.mutable(0);
			angleRoutine = Rotations.mutable(0);
			angularVelocityRoutine = RPM.mutable(0);
			drumRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
					voltage -> {
						shooter.setDrumVoltage(voltage.in(Volts));
					},
					log -> {
						log.motor("drum-shooter-motor")
							.voltage(appliedVoltsRoutine.mut_replace(shooterInputs.drumAppliedVolts))
							.angularPosition(angleRoutine.mut_replace(shooterInputs.drumPosition))
							.angularVelocity(angularVelocityRoutine.mut_replace(shooterInputs.drumVelocity));
					},
					this
				)
			);
			indexerRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
					voltage -> {
						shooter.setIndexerVoltage(voltage.in(Volts));
					},
					log -> {
						log.motor("indexer-motor")
							.voltage(appliedVoltsRoutine.mut_replace(shooterInputs.indexerAppliedVolts))
							.angularPosition(angleRoutine.mut_replace(shooterInputs.indexerPosition))
							.angularVelocity(angularVelocityRoutine.mut_replace(shooterInputs.indexerVelocity));
					},
					this
				)
			);
		} else {
			drumRoutine = null;
			indexerRoutine = null;
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
		ShooterIO shooter
	) {
		if (instance == null) {
			instance = new ShooterBase(
				shooter
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
			-fieldSpeeds.omegaRadiansPerSecond,
			fieldSpeeds.omegaRadiansPerSecond
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
	public Command shooterQuasiForward() {
		if (drumRoutine == null) return Commands.none();
		return drumRoutine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command indexerQuasiForward() {
		if (indexerRoutine == null) return Commands.none();
		return indexerRoutine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command shooterQuasiReverse() {
		if (drumRoutine == null) return Commands.none();
		return drumRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	public Command indexerQuasiReverse() {
		if (indexerRoutine == null) return Commands.none();
		return indexerRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	public Command shooterDynamicForward() {
		if (drumRoutine == null) return Commands.none();
		return drumRoutine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command indexerDynamicForward() {
		if (indexerRoutine == null) return Commands.none();
		return indexerRoutine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command shooterDynamicReverse() {
		if (drumRoutine == null) return Commands.none();
		return drumRoutine.dynamic(SysIdRoutine.Direction.kReverse);
	}

	public Command indexerDynamicReverse() {
		if (indexerRoutine == null) return Commands.none();
		return indexerRoutine.dynamic(SysIdRoutine.Direction.kReverse);
	}

	public Command unjamShooters() {
		return Commands.startEnd(
			() -> {
				shooter.setDrumSpeed(-1);
				shooter.setIndexerSpeed(1);
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
		shooter.stopDrum();
		shooter.stopIndexer();
	}

	public void setAddFivePercent(boolean b) {
		addFivePercent = b;
	}

	public void setSubFivePercent(boolean b) {
		subFivePercent = b;
	}

	@Override
	public void periodic() {
		shooter.updateInputs(shooterInputs);

		Logger.processInputs("ShooterBase", shooterInputs);

		Logger.recordOutput("AimedAtHub", isAimed);
		Logger.recordOutput("+5%", addFivePercent);
		Logger.recordOutput("-5%", subFivePercent);
		Logger.recordOutput("CurrentDistanceToHub", currentDistanceToTarget);
		Logger.recordOutput("TargetAngleToHub", targetAimAngleRad);
	}
}