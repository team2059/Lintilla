package org.team2059.Lintilla.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
	private static Conveyor instance;
	public final ConveyorIO io;
	public final ConveyorIOInputsAutoLogged inputs;

	private Conveyor(ConveyorIO io) {
		this.io = io;
		inputs = new ConveyorIOInputsAutoLogged();
	}

	public static Conveyor getInstance() {
		if (instance == null) {
			throw new RuntimeException("Conveyor is not initialized! Call initialize() first");
		}

		return instance;
	}

	public static void initialize(ConveyorIO io) {
		if (instance == null) {
			instance = new Conveyor(io);
		}
	}

	public Command conveyorIn() {
		return Commands.startEnd(
		  () -> io.setConveyorSpeed(ConveyorConstants.INTAKING_CONVEYOR_SPEED),
		  io::stopConveyor
		);
	}

	public Command conveyorOut() {
		return Commands.startEnd(
		  () -> io.setConveyorSpeed(-ConveyorConstants.INTAKING_CONVEYOR_SPEED),
		  io::stopConveyor
		);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Conveyor", inputs);
	}
}
