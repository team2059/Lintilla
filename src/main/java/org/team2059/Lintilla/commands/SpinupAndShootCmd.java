package org.team2059.Lintilla.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;

import static edu.wpi.first.units.Units.RPM;
import static org.team2059.Lintilla.Constants.ShooterConstants.spinupToleranceRpm;

/**
 * Spins up one shooter at a desired RPM, waiting for the spinup before running the indexer.
 *
 * Basic outline.
 *  *
 *  * 1. Shooter spins up.
 *  * 2. When shooter is within a tolerance of target RPM, spin indexer at 50% of target RPM.
 *  *
 *  * End (when interrupted):
 *  * -> Stop all motors immediately.
 */
public class SpinupAndShootCmd extends Command {
	private final ShooterBase shooterBase;

	// Holds our desired speeds
	private final double desiredRPM;

	// Controls whether right or left shooter is being run
	private final boolean runLeft;

	public SpinupAndShootCmd(ShooterBase shooterBase, double desiredRPM, boolean runLeft) {
		this.shooterBase = shooterBase;
		this.desiredRPM = desiredRPM;
		this.runLeft = runLeft;

		addRequirements(this.shooterBase);
	}

	@Override
	public void initialize() {
		// Spin up shooter
		if (runLeft) {
			shooterBase.leftShooter.setFlywheelRpm(desiredRPM);
		} else {
			shooterBase.rightShooter.setFlywheelRpm(desiredRPM);
		}
	}

	@Override
	public void execute() {
		if (runLeft && Math.abs(shooterBase.leftShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm) {
			// Left shooter is within tolerance. Spin indexer at 50% of target RPM
			shooterBase.leftShooter.setIndexerRpm(desiredRPM / 2);
		} else if (Math.abs(shooterBase.rightShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm) {
			// Right shooter is within tolerance. Spin indexer at 50% of target RPM
			shooterBase.rightShooter.setIndexerRpm(desiredRPM / 2);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		if (runLeft) {
			shooterBase.leftShooter.stopIndexer();
			shooterBase.leftShooter.stopFlywheel();
		} else {
			shooterBase.rightShooter.stopIndexer();
			shooterBase.rightShooter.stopFlywheel();
		}
	}
}
