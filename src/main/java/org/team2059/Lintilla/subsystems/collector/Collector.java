// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla.subsystems.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.Constants.CollectorConstants;

import static edu.wpi.first.units.Units.Rotations;

public class Collector extends SubsystemBase {

	public CollectorIO io;
	public CollectorIOInputsAutoLogged inputs;

	/**
	 * Creates a new Collector.
	 */
	public Collector(CollectorIO io) {
		this.io = io;
		inputs = new CollectorIOInputsAutoLogged();
	}

	/**
	 * @return Command which sets the tilt position to the outward position, then stops once tolerance reached.
	 */
	public Command collectorOut() {
		return this.runOnce(() -> io.setTiltPosition(CollectorConstants.thruBoreOut))
		  .andThen(Commands.waitUntil(() -> inputs.tiltPosition.isNear(Rotations.of(CollectorConstants.thruBoreOut), 0.05)))
		  .andThen(this.runOnce(() -> io.stopTilt()));
	}

	/**
	 * @return Command which sets the tilt position to the inward position, then stops once tolerance reached.
	 */
	public Command collectorIn() {
		return this.runOnce(() -> io.setTiltPosition(CollectorConstants.thruBoreIn))
		  .andThen(Commands.waitUntil(() -> inputs.tiltPosition.isNear(Rotations.of(CollectorConstants.thruBoreIn), 0.05)))
		  .andThen(this.runOnce(() -> io.stopTilt()));
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Collector", inputs);
	}
}
