package org.team2059.Lintilla.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
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

	// Variables below are used exclusively for SysID routine logging.
	private final MutVoltage appliedVoltsRoutine = Volts.mutable(0);
	private final MutAngle angleRoutine = Rotations.mutable(0);
	private final MutAngularVelocity angularVelocityRoutine = RPM.mutable(0);

	public ShooterBase(
	  ShooterIO leftShooter,
	  ShooterIO rightShooter
	) {
		this.leftShooter = leftShooter;
		this.rightShooter = rightShooter;

		// Declare SysID routines
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
	}

	// SysID getters
	public Command leftShooterQuasiForward() { return leftFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kForward); }
	public Command rightShooterQuasiForward() { return rightFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kForward); }
	public Command leftIndexerQuasiForward() { return leftIndexerRoutine.quasistatic(SysIdRoutine.Direction.kForward); }
	public Command rightIndexerQuasiForward() { return rightIndexerRoutine.quasistatic(SysIdRoutine.Direction.kForward); }

	public Command leftShooterQuasiReverse() { return leftFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kReverse); }
	public Command rightShooterQuasiReverse() { return rightFlywheelRoutine.quasistatic(SysIdRoutine.Direction.kReverse); }
	public Command leftIndexerQuasiReverse() { return leftIndexerRoutine.quasistatic(SysIdRoutine.Direction.kReverse); }
	public Command rightIndexerQuasiReverse() { return rightIndexerRoutine.quasistatic(SysIdRoutine.Direction.kReverse); }

	public Command leftShooterDynamicForward() { return leftFlywheelRoutine.dynamic(SysIdRoutine.Direction.kForward); }
	public Command rightShooterDynamicForward() { return rightFlywheelRoutine.dynamic(SysIdRoutine.Direction.kForward); }
	public Command leftIndexerDynamicForward() { return leftIndexerRoutine.dynamic(SysIdRoutine.Direction.kForward); }
	public Command rightIndexerDynamicForward() { return rightIndexerRoutine.dynamic(SysIdRoutine.Direction.kForward); }

	public Command leftShooterDynamicReverse() { return leftFlywheelRoutine.dynamic(SysIdRoutine.Direction.kReverse); }
	public Command rightShooterDynamicReverse() { return rightFlywheelRoutine.dynamic(SysIdRoutine.Direction.kReverse); }
	public Command leftIndexerDynamicReverse() { return leftIndexerRoutine.dynamic(SysIdRoutine.Direction.kReverse); }
	public Command rightIndexerDynamicReverse() { return rightIndexerRoutine.dynamic(SysIdRoutine.Direction.kReverse); }

	/**
	 * Calculate the needed velocity (in meters per second) of the fuel as it exits
	 * @param d horizontal magnitude (along floor only, in meters)
	 * @return velocity in meters/sec
	 */
	public double getTargetFuelVelocityMps(double d) {
		// Ensure minimum shot, won't cause NaN
		if (d <= ShooterConstants.minimumShotDistanceMeters) {
			return 0.0;
		}

		return Math.sqrt(
		  (ShooterConstants.gravitationalAccelerationMpss * d * d)
			/ (2 * ShooterConstants.cosineShooterAngleSquared * ((d * ShooterConstants.tangentShooterAngle) - ShooterConstants.dY))
		);
	}

	public void stopAllSubsystemMotors() {
		leftShooter.stopFlywheel();
		leftShooter.stopIndexer();
		rightShooter.stopFlywheel();
		rightShooter.stopIndexer();
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

	public Command setShooterRPM5000() {
		return Commands.startEnd(
		  () -> {
			  leftShooter.setFlywheelRpm(5000);
			  rightShooter.setFlywheelRpm(5000);
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
	}
}
