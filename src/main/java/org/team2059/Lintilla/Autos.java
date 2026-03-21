package org.team2059.Lintilla;

import com.pathplanner.lib.auto.NamedCommands;
import org.team2059.Lintilla.commands.SpinupAndShootCommand;

import static org.team2059.Lintilla.RobotContainer.*;

/**
 * Class that holds all autonomous commands
 */
public final class Autos {
	public static void registerNamedCommands() {
		NamedCommands.registerCommand(
		  "ShootWithDistance",
		  new SpinupAndShootCommand(drivetrain, shooterBase, conveyor)
		);

		NamedCommands.registerCommand(
		  "CollectorOutInfinite",
		  collector.tiltOutInfinite()
		);

		NamedCommands.registerCommand(
		  "CollectorInInfinite",
		  collector.tiltInInfinite()
		);

		NamedCommands.registerCommand(
		  "CollectorOut",
		  collector.tiltOut()
		);

		NamedCommands.registerCommand(
		  "CollectorIn",
		  collector.tiltIn()
		);

		NamedCommands.registerCommand(
		  "Intake",
		  collector.intake()
		);
	}
}
