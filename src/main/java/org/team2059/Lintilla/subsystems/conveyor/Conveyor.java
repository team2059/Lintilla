package org.team2059.Lintilla.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team2059.Lintilla.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
    public final ConveyorIO io;
    public final ConveyorIOInputsAutoLogged inputs;

    public Conveyor(ConveyorIO io) {
        this.io = io;
        inputs = new ConveyorIOInputsAutoLogged();
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
}
