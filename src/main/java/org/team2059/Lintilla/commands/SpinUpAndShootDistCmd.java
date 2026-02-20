package org.team2059.Lintilla.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.RobotContainer;
import org.team2059.Lintilla.subsystems.collector.Collector;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;

import static edu.wpi.first.units.Units.RPM;
import static org.team2059.Lintilla.Constants.ShooterConstants.spinupToleranceRpm;

import org.littletonrobotics.junction.Logger;

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
public class SpinUpAndShootDistCmd extends Command {
	private final ShooterBase shooterBase;
	private final Collector collector;

	// Holds our desired speeds
	private final double desiredRPM;

	public SpinUpAndShootDistCmd(ShooterBase shooterBase, Collector collector, double distance) {
		this.shooterBase = shooterBase;
		this.collector = collector;
		this.desiredRPM = shooterBase.getTargetFuelRpm(distance);

		addRequirements(this.shooterBase);
	}

	@Override
	public void initialize() {
		// Spin up shooters	
		shooterBase.leftShooter.setFlywheelRpm(desiredRPM);
		shooterBase.rightShooter.setFlywheelRpm(desiredRPM);
	}

	@Override
	public void execute() {
        Logger.recordOutput("desiredRPMs", desiredRPM);
		if (Math.abs(shooterBase.leftShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm) {
			// Left shooter is within tolerance. Spin indexer
			shooterBase.leftShooter.setIndexerSpeed(0.5);
			collector.io.runConveyor(0.25);
		}
		if (Math.abs(shooterBase.rightShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm) {
			// Right shooter is within tolerance. Spin indexer
			shooterBase.rightShooter.setIndexerSpeed(0.5);
			collector.io.runConveyor(0.25);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		shooterBase.leftShooter.stopIndexer();
		shooterBase.leftShooter.stopFlywheel();
		shooterBase.rightShooter.stopIndexer();
		shooterBase.rightShooter.stopFlywheel();
		collector.io.stopConveyor();
	}
}

