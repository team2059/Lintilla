// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla.subsystems.collector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Collector extends SubsystemBase {
	public CollectorIO collectorIO;
	public CollectorIOInputsAutoLogged collectorInputs;

	/**
	 * Creates a new Collector.
	 */
	public Collector(CollectorIO collectorIO) {
		this.collectorIO = collectorIO;
		collectorInputs = new CollectorIOInputsAutoLogged();
	}

	public Command setTiltPos(double position) {
		return Commands.run(() -> collectorIO.setTiltPosition(position));
	}

	// public Command intakeFuel() {
	// return Commands.run(() -> collectorIO.)
	// }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//SmartDashboard.putNumber("ThruBorePos", this.collectorIO.thruBoreEnc.getPosition());
		collectorIO.updateInputs(collectorInputs);
		Logger.processInputs("Collector", collectorInputs);
	}
}
