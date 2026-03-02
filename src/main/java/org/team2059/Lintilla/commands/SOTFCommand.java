package org.team2059.Lintilla.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.Constants;
import org.team2059.Lintilla.subsystems.collector.Collector;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;

import static edu.wpi.first.units.Units.RPM;
import static org.team2059.Lintilla.Constants.ShooterConstants.spinupToleranceRpm;

public class SOTFCommand extends Command {
	private final ShooterBase shooterBase;
	private final Collector collector;
	private final Drivetrain drivetrain;

	public SOTFCommand(Drivetrain drivetrain, ShooterBase shooterBase, Collector collector) {
		this.drivetrain = drivetrain;
		this.shooterBase = shooterBase;
		this.collector = collector;

		addRequirements(shooterBase);
	}

	@Override
	public void execute() {

		// Look up desired RPM based on distance to virtual target
		double desiredRPM = shooterBase.getTargetRpm(shooterBase.currentDistanceToTarget);

		shooterBase.leftShooter.setFlywheelRpm(desiredRPM);
		shooterBase.rightShooter.setFlywheelRpm(desiredRPM);

		// Check if RPMs are within tolerance
		boolean leftAtSpeed = Math.abs(shooterBase.leftShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm;
		boolean rightAtSpeed = Math.abs(shooterBase.rightShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm;

		// Fire if ready, otherwise stop feeding
		if (shooterBase.isAimed && Math.abs(shooterBase.leftShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm) {
			// Left shooter is within tolerance. Spin indexer
			shooterBase.leftShooter.setIndexerSpeed(Constants.ShooterConstants.indexerShootingSpeed);
			collector.io.runConveyor(Constants.ShooterConstants.conveyorShootingSpeed);
		}
		if (shooterBase.isAimed && Math.abs(shooterBase.rightShooterInputs.flywheelVelocity.in(RPM) - desiredRPM) <= spinupToleranceRpm) {
			// Right shooter is within tolerance. Spin indexer
			shooterBase.rightShooter.setIndexerSpeed(Constants.ShooterConstants.indexerShootingSpeed);
			collector.io.runConveyor(Constants.ShooterConstants.conveyorShootingSpeed);
		}
	}

	@Override
	public boolean isFinished() {
		return false; // Typically ends when the driver lets go of the button
	}

	@Override
	public void end(boolean interrupted) {
		shooterBase.leftShooter.stopFlywheel();
		shooterBase.rightShooter.stopFlywheel();
		shooterBase.leftShooter.stopIndexer();
		shooterBase.rightShooter.stopIndexer();
		collector.io.stopConveyor();
	}
}
