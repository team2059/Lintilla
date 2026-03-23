package org.team2059.Lintilla.subsystems.shooter;

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
import static org.team2059.Lintilla.Constants.OperatorConstants.SHOOTER_ADD5PERCENT_SWITCH;
import static org.team2059.Lintilla.Constants.OperatorConstants.tuningMode;
import static org.team2059.Lintilla.Constants.ShooterConstants;

public class ShooterBase extends SubsystemBase {

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
	public double currentDistanceToTarget = 0.0;

	public ShooterBase(
	  ShooterIO leftShooter,
	  ShooterIO rightShooter
	) {
		this.leftShooter = leftShooter;
		this.rightShooter = rightShooter;

		addFivePercent = !RobotContainer.buttonBox.getRawButton(SHOOTER_ADD5PERCENT_SWITCH);

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

	/**
	 * Fetch the needed RPM of the flywheel to shoot fuel a given distance
	 *
	 * @param distanceMeters horizontal distance to target, in meters
	 * @return RPM to set the shooter at
	 */
	public double getTargetRpm(double distanceMeters) {
		Logger.recordOutput("distanceMeters", distanceMeters);
		double rpm = ShooterConstants.SHOOTER_MAP.get(distanceMeters).rpm();
		Logger.recordOutput("desiredRPM", rpm);
		return rpm;
	}

	/**
	 * Fetch the estimated Time of Flight for shooting from a certain distance
	 *
	 * @param distanceMeters
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

	public Command runIndexerSpeed() {
		return Commands.startEnd(
		  () -> {
			  leftShooter.setIndexerSpeed(-0.65);
			  rightShooter.setIndexerSpeed(0.65);
		  },
		  () -> {
			  leftShooter.stopIndexer();
			  rightShooter.stopIndexer();
		  }
		);
	}

	public void setAddFivePercent(boolean b) {
		addFivePercent = b;
	}

	public Command setShooterRPM1500() {
		return Commands.startEnd(
		  () -> {
			  leftShooter.setFlywheelRpm(1500);
			  rightShooter.setFlywheelRpm(1500);
		  },
		  () -> {
			  leftShooter.stopFlywheel();
			  rightShooter.stopFlywheel();
		  }
		);
	}

	public Command setShooterRPM3000() {
		return Commands.startEnd(
		  () -> {
			  leftShooter.setFlywheelRpm(3000);
			  rightShooter.setFlywheelRpm(3000);
		  },
		  () -> {
			  leftShooter.stopFlywheel();
			  rightShooter.stopFlywheel();
		  }
		);
	}

	public Command setShooterRPM6000() {
		return Commands.startEnd(
		  () -> {
			  leftShooter.setFlywheelRpm(6000);
			  rightShooter.setFlywheelRpm(6000);
		  },
		  () -> {
			  leftShooter.stopFlywheel();
			  rightShooter.stopFlywheel();
		  }
		);
	}

	public Command runIndexer() {
		return Commands.startEnd(
		  () -> {
			  leftShooter.setIndexerRpm(1000);
			  rightShooter.setIndexerRpm(1000);
		  },
		  () -> {
			  leftShooter.stopIndexer();
			  rightShooter.stopIndexer();
		  }
		);
	}

	@Override
	public void periodic() {
		leftShooter.updateInputs(leftShooterInputs);
		rightShooter.updateInputs(rightShooterInputs);

		Logger.processInputs("ShooterBase/Left", leftShooterInputs);
		Logger.processInputs("ShooterBase/Right", rightShooterInputs);

		Logger.recordOutput("IsAimed", isAimed);
		Logger.recordOutput("AddFivePercent", addFivePercent);
	}
}
