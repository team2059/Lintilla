package org.team2059.Lintilla.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.subsystems.collector.Collector;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;
import org.team2059.Lintilla.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.RPM;
import static org.team2059.Lintilla.Constants.ShooterConstants.spinupToleranceRpm;

/**
 * Spinup and shoot command with tunable RPM. Useful for data collection.
 */
public class ShooterDataCollectionCmd extends Command {
	private final ShooterBase shooterBase;
	private final Collector collector;

	// Holds our desired speeds
	private final LoggedTunableNumber speedRPM = new LoggedTunableNumber("speedRPM", 0.0);

	public ShooterDataCollectionCmd(ShooterBase shooterBase, Collector collector) {
		this.shooterBase = shooterBase;
		this.collector = collector;

		addRequirements(this.shooterBase);
	}

	@Override
	public void initialize() {
		// Spin up shooters
		shooterBase.leftShooter.setFlywheelRpm(speedRPM.get());
		shooterBase.rightShooter.setFlywheelRpm(speedRPM.get());
	}

	@Override
	public void execute() {
		if (Math.abs(shooterBase.leftShooterInputs.flywheelVelocity.in(RPM) - speedRPM.get()) <= spinupToleranceRpm) {
			// Left shooter is within tolerance. Spin indexer
			shooterBase.leftShooter.setIndexerSpeed(0.5);
			collector.io.runConveyor(0.5);
		}
		if (Math.abs(shooterBase.rightShooterInputs.flywheelVelocity.in(RPM) - speedRPM.get()) <= spinupToleranceRpm) {
			// Right shooter is within tolerance. Spin indexer
			shooterBase.rightShooter.setIndexerSpeed(0.5);
			collector.io.runConveyor(0.5);
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
