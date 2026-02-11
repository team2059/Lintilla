// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla.subsystems.collector;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.Constants.CollectorConstants;

import static edu.wpi.first.units.Units.*;

public class Collector extends SubsystemBase {

	public final CollectorIO io;
	public final CollectorIOInputsAutoLogged inputs;

	// Declare all stuff for a SysID routine
	private final SysIdRoutine routine;
	private final MutVoltage appliedVoltageRoutine = Volts.mutable(0);
	private final MutAngle angleRoutine = Rotations.mutable(0);
	private final MutAngularVelocity angularVelocityRoutine = RPM.mutable(0);

	/**
	 * Creates a new Collector.
	 */
	public Collector(CollectorIO io) {
		this.io = io;
		inputs = new CollectorIOInputsAutoLogged();

		// Configure SysID routine
		routine = new SysIdRoutine(
		  new SysIdRoutine.Config(
			Volts.of(0.5).per(Second), // Ramp rate in volts per second
			Volts.of(2), // Dynamic step voltage
			Time.ofBaseUnits(4, Second), // Test duration in seconds
			null
		  ),
		  new SysIdRoutine.Mechanism(
			voltage -> {
				io.setTiltVolts(voltage.in(Volts));
			},
			log -> {
				log.motor("collector-tiltmotor")
				  .voltage(appliedVoltageRoutine.mut_replace(inputs.tiltAppliedVolts))
				  .angularPosition(angleRoutine.mut_replace(inputs.tiltPosition))
				  .angularVelocity(angularVelocityRoutine.mut_replace(inputs.tiltVelocity));
			},
			this
		  )
		);
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
		return this.runOnce(() -> io.setTiltPosition(CollectorConstants.thruBoreIn)) // Set reference for PID/FF controller
		  .andThen(Commands.waitUntil(() -> inputs.tiltPosition.isNear(Rotations.of(CollectorConstants.thruBoreIn), 0.05))) // Keep checking how close we are
		  .andThen(this.runOnce(() -> io.stopTilt())); // Stop when close enough
	}

	public Command sysIdQuasiForward() {
		return routine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command sysIdQuasiReverse() {
		return routine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	public Command sysIdDynamicForward() {
		return routine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command sysIdDynamicReverse() {
		return routine.dynamic(SysIdRoutine.Direction.kReverse);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Collector", inputs);
	}
}
