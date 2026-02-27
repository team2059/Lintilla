package org.team2059.Lintilla.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.Constants;
import org.team2059.Lintilla.subsystems.collector.Collector;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;

import static edu.wpi.first.units.Units.RPM;
import static org.team2059.Lintilla.Constants.ShooterConstants.spinupToleranceRpm;

/**
 * Spin up & shoot command, calculates distance to alliance Hub and looks up RPM.
 */
public class SpinupAndShootCmd extends Command {
	private final ShooterBase shooterBase;
	private final Collector collector;
	private final Drivetrain drivetrain;

	// Holds our desired speeds
	private double desiredRPM;
	private boolean desiredRPMHardcoded;

	public SpinupAndShootCmd(Drivetrain drivetrain, ShooterBase shooterBase, Collector collector) {
		this.shooterBase = shooterBase;
		this.drivetrain = drivetrain;
		this.collector = collector;

		this.desiredRPM = shooterBase.getTargetRpm(drivetrain.calculateDistanceShooterToHubMeters());
		this.desiredRPMHardcoded = false;

		addRequirements(this.shooterBase);
	}

	public SpinupAndShootCmd(Drivetrain drivetrain, ShooterBase shooterBase, Collector collector, double desiredRPM) {
		this.shooterBase = shooterBase;
		this.drivetrain = drivetrain;
		this.collector = collector;

		this.desiredRPM = desiredRPM;
		this.desiredRPMHardcoded = true;

		addRequirements(this.shooterBase);
	}

	@Override
	public void initialize() {

		if (!desiredRPMHardcoded) {
			// Calculate desired RPM based on distance from alliance Hub
			desiredRPM = shooterBase.getTargetRpm(drivetrain.calculateDistanceShooterToHubMeters());
		}

		// Spin up shooters	
		shooterBase.leftShooter.setFlywheelRpm(desiredRPM);
		shooterBase.rightShooter.setFlywheelRpm(desiredRPM);
	}

	@Override
	public void execute() {
		if (Math.abs(shooterBase.leftShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm) {
			// Left shooter is within tolerance. Spin indexer
			shooterBase.leftShooter.setIndexerSpeed(Constants.ShooterConstants.indexerShootingSpeed);
			collector.io.runConveyor(Constants.ShooterConstants.conveyorShootingSpeed);
		}
		if (Math.abs(shooterBase.rightShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm) {
			// Right shooter is within tolerance. Spin indexer
			shooterBase.rightShooter.setIndexerSpeed(Constants.ShooterConstants.indexerShootingSpeed);
			collector.io.runConveyor(Constants.ShooterConstants.conveyorShootingSpeed);
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
