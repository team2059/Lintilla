package org.team2059.Lintilla;

import com.pathplanner.lib.auto.NamedCommands;
import org.team2059.Lintilla.commands.SpinupAndShootCommand;
import org.team2059.Lintilla.subsystems.collector.Collector;
import org.team2059.Lintilla.subsystems.conveyor.Conveyor;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;

/**
 * Class that holds all autonomous commands
 */
public final class Autos {
	public static void registerNamedCommands() {
		NamedCommands.registerCommand(
		  "ShootWithDistance",
		  new SpinupAndShootCommand(Drivetrain.getInstance(), ShooterBase.getInstance(), Conveyor.getInstance()).alongWith(Collector.getInstance().agitationCommand())
		);

		NamedCommands.registerCommand(
		  "CollectorOutInfinite",
		  Collector.getInstance().tiltOutInfinite()
		);

		NamedCommands.registerCommand(
		  "CollectorInInfinite",
		  Collector.getInstance().tiltInInfinite()
		);

		NamedCommands.registerCommand(
		  "CollectorOut",
		  Collector.getInstance().tiltOut()
		);

		NamedCommands.registerCommand(
		  "CollectorIn",
		  Collector.getInstance().tiltIn()
		);

		NamedCommands.registerCommand(
		  "Intake",
		  Collector.getInstance().intake()
		);
	}
}
