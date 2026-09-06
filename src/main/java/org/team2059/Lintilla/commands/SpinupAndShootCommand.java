package org.team2059.Lintilla.commands;

import static edu.wpi.first.units.Units.RPM;
import static org.team2059.Lintilla.Constants.ConveyorConstants.SHOOTING_CONVEYOR_SPEED;
import static org.team2059.Lintilla.Constants.ShooterConstants.INDEXER_RPM_WHILE_SHOOTING;
import static org.team2059.Lintilla.Constants.ShooterConstants.INDEXER_SPEED_WHILE_SHOOTING;
import static org.team2059.Lintilla.Constants.ShooterConstants.SPINUP_TOLERANCE_RPM;

import java.rmi.AccessException;

import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.Constants.ShooterConstants;
import org.team2059.Lintilla.subsystems.conveyor.Conveyor;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;
import org.team2059.Lintilla.util.LoggedTunableNumber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to spin the shooters up at a specific velocity (or calculate velocity based on distance to hub) and shoot
 * when a certain tolerance is reached.
 */
public class SpinupAndShootCommand extends Command {
	private static final double SPINUP_TIME_SECONDS = 1.5;
	private final ShooterBase shooterBase;
	private final Conveyor conveyor;
	private double initialDesiredRPM;
	private double desiredRPM; // The actual RPM to push to the shooters
	private boolean desiredRPMHardcoded; // Whether or not we're using distance-calculated RPM
	private Timer spinUpTimer = new Timer();
	private Timer shooterAcceleratedTimer = new Timer();
	private final LoggedTunableNumber kP = new LoggedTunableNumber("DrumkP", ShooterConstants.FLYWHEEL_P);
	private final LoggedTunableNumber kV = new LoggedTunableNumber("DrumkV", ShooterConstants.FLYWHEEL_V);
	private final LoggedTunableNumber tunableRPM = new LoggedTunableNumber("RPM", desiredRPM);

	private boolean accelerated = true;
	private boolean hitAcceleratedSetpoint = false;

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

		this.initialDesiredRPM = 0; // This will be set later
		this.desiredRPMHardcoded = false;

		addRequirements(this.shooterBase, this.conveyor);

		System.out.println("Shooting distance");
	}

	/**
	 * Constructor for shots at specific RPMs
	 *
	 * @param shooterBase the ShooterBase subsystem
	 * @param conveyor    the Conveyor subsystem
	 * @param initialDesiredRPM  the desired speeds in RPM
	 */
	public SpinupAndShootCommand(
	  ShooterBase shooterBase,
	  Conveyor conveyor,
	  double initialDesiredRPM
	) {
		this.shooterBase = shooterBase;
		this.conveyor = conveyor;

		this.initialDesiredRPM = initialDesiredRPM;
		this.desiredRPMHardcoded = true;

		addRequirements(this.shooterBase, this.conveyor);

		System.out.println("Shooting hardcoded");
	}

	@Override
	public void initialize() {
		spinUpTimer.restart();
		shooterAcceleratedTimer.restart();

		// Process desiredRPM for hardcoded shots
		if (desiredRPMHardcoded) {
			LoggedTunableNumber.ifChanged(
				hashCode(),
				() -> {
					desiredRPM = tunableRPM.get();
				},
				tunableRPM
			);

			initialDesiredRPM = desiredRPM;
			
			// Check switches
			if (shooterBase.subFivePercent) {
				desiredRPM = initialDesiredRPM * 0.95;
			} else if (shooterBase.addFivePercent) {
				desiredRPM = initialDesiredRPM * 1.05;
			} else {
				desiredRPM = initialDesiredRPM;
			}
		}
	}

	@Override
	public void execute() {
		LoggedTunableNumber.ifChanged(
			hashCode(), 
			() -> {
			shooterBase.shooter.setDrumkP(kP.get());
			shooterBase.shooter.setDrumkV(kV.get());
			}, 
			kP, kV
		);


		if (!desiredRPMHardcoded) {
			// Process desiredRPM for distance-based shots
			

			// Calculate the latest SOTF numbers
			shooterBase.calculateSOTF(Drivetrain.getInstance().getEstimatedPose(), Drivetrain.getInstance().getFieldRelativeSpeeds());

			// Fetch the latest distance.
			initialDesiredRPM = shooterBase.getTargetRpm(shooterBase.currentDistanceToTarget);

			// Check the switches
			if (shooterBase.subFivePercent) {
				desiredRPM = initialDesiredRPM * 0.95;
			} else if (shooterBase.addFivePercent) {
				desiredRPM = initialDesiredRPM * 1.05;
			} else {
				desiredRPM = initialDesiredRPM;

			}
		}

		Logger.recordOutput("desiredRPM", desiredRPM);

		if (desiredRPM < 100) this.cancel();

		double acceleratedRPM = desiredRPM * 1.2;

		// Set the flywheel to the desired RPM, whether it's hardcoded or
		// not, it doesn't matter at this point in execution.

		if (accelerated) {
			shooterBase.shooter.setDrumRpm(acceleratedRPM);
		} else {
			shooterBase.shooter.setDrumRpm(desiredRPM);
		}

		double drumRPM = shooterBase.shooterInputs.drumVelocity.in(RPM);

		if (accelerated) {
			if (Math.abs(drumRPM - acceleratedRPM) <= SPINUP_TOLERANCE_RPM) {
				shooterBase.shooter.setIndexerRpm(0.75 * desiredRPM);
				conveyor.io.setConveyorSpeed(SHOOTING_CONVEYOR_SPEED);
			}
		} else{
			if (Math.abs(drumRPM - desiredRPM) <= SPINUP_TOLERANCE_RPM) {
				shooterBase.shooter.setIndexerRpm(0.75 * desiredRPM);
				conveyor.io.setConveyorSpeed(SHOOTING_CONVEYOR_SPEED);
			}
		}


		if (Math.abs(drumRPM - acceleratedRPM) <= SPINUP_TOLERANCE_RPM && accelerated && !hitAcceleratedSetpoint) {
			shooterAcceleratedTimer.reset();
			hitAcceleratedSetpoint = true;
		}

		if (shooterAcceleratedTimer.hasElapsed(0.4) && hitAcceleratedSetpoint) {
			accelerated = false;
		}
	}

	public void resetAcceleratedBooleans() {
		accelerated = true;
		hitAcceleratedSetpoint = false;
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

		resetAcceleratedBooleans();

		// Stop everything that we used
		shooterBase.shooter.stopIndexer();
		shooterBase.shooter.stopDrum();
		conveyor.io.stopConveyor();
	}
}
