package org.team2059.Lintilla.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.subsystems.conveyor.Conveyor;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;

import static edu.wpi.first.units.Units.RPM;
import static org.team2059.Lintilla.Constants.ConveyorConstants.SHOOTING_CONVEYOR_SPEED;
import static org.team2059.Lintilla.Constants.ShooterConstants.INDEXER_SPEED_WHILE_SHOOTING;
import static org.team2059.Lintilla.Constants.ShooterConstants.SPINUP_TOLERANCE_RPM;

/**
 * Command to spin the shooters up at a specific velocity (or calculate velocity based on distance to hub) and shoot
 * when a certain tolerance is reached.
 */
public class SpinupAndShootCommand extends Command {
	private static final double SPINUP_TIME_SECONDS = 1.5;
	private final ShooterBase shooterBase;
	private final Conveyor conveyor;
	private double desiredRPM; // The actual RPM to push to the shooters
	private boolean desiredRPMHardcoded; // Whether or not we're using distance-calculated RPM
	private Timer spinUpTimer = new Timer();

	/**
	 * Constructor for distance-based shots (shoots on the fly)
	 *
	 * @param shooterBase the ShooterBase subsystem
	 * @param conveyor    the Conveyor subsystem
	 */
	public SpinupAndShootCommand(
	  ShooterBase shooterBase,
	  Conveyor conveyor
	) {
		this.shooterBase = shooterBase;
		this.conveyor = conveyor;

		this.desiredRPM = 0; // This will be set later
		this.desiredRPMHardcoded = false;

		addRequirements(this.shooterBase, this.conveyor);
	}

	/**
	 * Constructor for shots at specific RPMs
	 *
	 * @param shooterBase the ShooterBase subsystem
	 * @param conveyor    the Conveyor subsystem
	 * @param desiredRPM  the desired speeds in RPM
	 */
	public SpinupAndShootCommand(
	  ShooterBase shooterBase,
	  Conveyor conveyor,
	  double desiredRPM
	) {
		this.shooterBase = shooterBase;
		this.conveyor = conveyor;

		this.desiredRPM = desiredRPM;
		this.desiredRPMHardcoded = true;

		addRequirements(this.shooterBase, this.conveyor);
	}

	@Override
	public void initialize() {
		spinUpTimer.reset();
	}

	@Override
	public void execute() {

		if (!desiredRPMHardcoded) {
			// We're not hardcoded.

			// Calculate the latest SOTF numbers
			shooterBase.calculateSOTF(Drivetrain.getInstance().getEstimatedPose(), Drivetrain.getInstance().getFieldRelativeSpeeds());

			// Fetch the latest distance.
			desiredRPM = shooterBase.getTargetRpm(shooterBase.currentDistanceToTarget);
		}

		// Check the switch for +5%
		desiredRPM = (shooterBase.addFivePercent)
		  ? desiredRPM * 1.05
		  : desiredRPM;

		// Check the switch for -5%
		desiredRPM = (shooterBase.subFivePercent)
		  ? desiredRPM * 0.95
		  : desiredRPM;

		Logger.recordOutput("desiredRPM", desiredRPM);

		if (desiredRPM < 100) this.cancel();

		// Set the two flywheels to the desired RPM, whether it's hardcoded or
		// not, it doesn't matter at this point in execution.
		shooterBase.leftShooter.setFlywheelRpm(desiredRPM);
		shooterBase.rightShooter.setFlywheelRpm(desiredRPM);

		double leftRPM = shooterBase.leftShooterInputs.flywheelVelocity.in(RPM);
		double rightRPM = shooterBase.rightShooterInputs.flywheelVelocity.in(RPM);

		if (
		  // Shooters within tolerance, or timer has run out
		  (Math.abs(leftRPM - desiredRPM) <= SPINUP_TOLERANCE_RPM && Math.abs(rightRPM - desiredRPM) <= SPINUP_TOLERANCE_RPM)
			|| spinUpTimer.hasElapsed(SPINUP_TIME_SECONDS)
		) {
			// Spin indexers and conveyor
			shooterBase.leftShooter.setIndexerSpeed(INDEXER_SPEED_WHILE_SHOOTING);
			shooterBase.rightShooter.setIndexerSpeed(INDEXER_SPEED_WHILE_SHOOTING);
			conveyor.io.setConveyorSpeed(SHOOTING_CONVEYOR_SPEED);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			System.out.println("RPM was " + desiredRPM);
		}

		// Stop everything that we used
		shooterBase.leftShooter.stopIndexer();
		shooterBase.leftShooter.stopFlywheel();
		shooterBase.rightShooter.stopIndexer();
		shooterBase.rightShooter.stopFlywheel();
		conveyor.io.stopConveyor();
	}
}
