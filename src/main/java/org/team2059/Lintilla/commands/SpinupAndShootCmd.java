package org.team2059.Lintilla.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;

/*
 * Basic outline.
 *
 * 1. Shooters spin up.
 * 2. When shooters are within a tolerance of target RPM, spin indexer at 50% of target RPM.
 *
 * End (when interrupted):
 * -> Stop all motors immediately.
 */

public class SpinupAndShootCmd extends Command {
	private final ShooterBase shooterBase;

	private boolean runLeft;

	public SpinupAndShootCmd(ShooterBase shooterBase, boolean runLeft) {
		this.shooterBase = shooterBase;
		this.runLeft = runLeft;

		addRequirements(this.shooterBase);
	}

	@Override
	public void initialize() {
		// Spin up shooter

	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
