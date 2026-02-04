package org.team2059.Lintilla.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;

/*
 * Basic outline.
 *
 * 1. Shooters spin up.
 * 2. When shooters are within a tolerance of target RPM, spin indexer at 50% of target RPM.
 */

public class SpinupAndShootCmd extends Command {
	private final ShooterBase shooterBase;

	public SpinupAndShootCmd(ShooterBase shooterBase) {
		this.shooterBase = shooterBase;
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.shooterBase);
	}

	@Override
	public void initialize() {

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
