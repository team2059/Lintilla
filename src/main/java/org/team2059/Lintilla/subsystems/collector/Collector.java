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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static org.team2059.Lintilla.Constants.CollectorConstants.*;
import static org.team2059.Lintilla.Constants.OperatorConstants.tuningMode;

public class Collector extends SubsystemBase {

	private static Collector instance;
	public final CollectorIO io;
	public final CollectorIOInputsAutoLogged inputs;
	// Declare all stuff for a SysID routine
	private final SysIdRoutine routine;
	private final MutVoltage appliedVoltageRoutine;
	private final MutAngle angleRoutine;
	private final MutAngularVelocity angularVelocityRoutine;
	/**
	 * Creates a new Collector.
	 */
	private Collector(CollectorIO io) {
		this.io = io;
		inputs = new CollectorIOInputsAutoLogged();

		if (tuningMode) {
			// Configure SysID routine
			appliedVoltageRoutine = Volts.mutable(0);
			angleRoutine = Rotations.mutable(0);
			angularVelocityRoutine = RPM.mutable(0);
			routine = new SysIdRoutine(
			  new SysIdRoutine.Config(
				Volts.of(1).per(Second), // Ramp rate in volts per second
				Volts.of(2), // Dynamic step voltage
				Time.ofBaseUnits(6, Second), // Test duration in seconds
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
		} else {
			appliedVoltageRoutine = null;
			angleRoutine = null;
			angularVelocityRoutine = null;
			routine = null;
		}
	}

	public static Collector getInstance() {
		if (instance == null) {
			throw new RuntimeException("Collector is not initialized! Call initialize() first");
		}

		return instance;
	}

	public static void initialize(CollectorIO io) {
		if (instance == null) {
			instance = new Collector(io);
		}
	}

	/**
	 * @return Command which sets tilt position to a given setpoint, stops tilt when within tolerance
	 */
	public Command tiltToSetpoint(double setpoint) {
		return this.run(() -> io.setTiltPosition(setpoint))
		  .until(() -> Math.abs(inputs.tiltPosition.in(Rotations) - setpoint) <= TILT_TOLERANCE_ROTATIONS)
		  .finallyDo(io::stopTilt);
	}

	/**
	 * @return Command which sets tilt position to OUTward, stops tilt when within tolerance
	 */
	public Command tiltOut() {
		return this.run(() -> io.setTiltPosition(THRUBORE_OUT))
		  .until(() -> Math.abs(inputs.tiltPosition.in(Rotations) - THRUBORE_OUT) <= TILT_TOLERANCE_ROTATIONS)
		  .finallyDo(io::stopTilt);
	}

	public Command tiltOutInfinite() {
		return this.startEnd(
		  () -> io.setTiltPosition(THRUBORE_OUT),
		  () -> io.stopTilt()
		);
	}

	/**
	 * @return Command which sets tilt position to INward, stops tilt when within tolerance
	 */
	public Command tiltIn() {
		return this.run(() -> io.setTiltPosition(THRUBORE_IN))
		  .until(() -> Math.abs(inputs.tiltPosition.in(Rotations) - THRUBORE_IN) <= TILT_TOLERANCE_ROTATIONS)
		  .finallyDo(io::stopTilt);
	}

	public Command tiltInInfinite() {
		return this.startEnd(
		  () -> io.setTiltPosition(THRUBORE_IN),
		  () -> io.stopTilt()
		);
	}

	/**
	 * Runs rollers in the intaking direction
	 */
	public Command intake() {
		return Commands.startEnd(
		  () -> io.setIntakeSpeed(INTAKING_ROLLER_SPEED),
		  io::stopIntake
		);
	}

	/**
	 * Runs rollers in the outtaking (unjam) direction
	 */
	public Command outtake() {
		return Commands.startEnd(
		  () -> io.setIntakeSpeed(OUTTAKING_ROLLER_SPEED),
		  io::stopIntake
		);
	}

	/**
	 * Runs tilt out and intake commands in parallel
	 */
	public Command tiltOutAndIntake() {
		return new ParallelCommandGroup(
		  tiltOut(),
		  intake()
		);
	}

	// Agitation command to help prevent jams, runs intake up and down repeatedly
	// From Team 5667: https://github.com/NAHSRobotics-Team5667/FRC-2026/blob/main/src/main/java/frc/robot/subsystems/IntakeSubsystem.java
	public Command agitationCommand() {
		return
		  Commands.waitSeconds(1).andThen(
			Commands.sequence(
				Commands.waitSeconds(0.5),
				tiltIn().withTimeout(1),
				Commands.waitSeconds(0.12),
				tiltOut().withTimeout(1),
				Commands.waitSeconds(0.12))
			  .repeatedly().alongWith(intake())
		  );
	}

	public Command sysIdQuasiForward() {
		if (routine == null) return Commands.none();
		return routine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command sysIdQuasiReverse() {
		if (routine == null) return Commands.none();
		return routine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	public Command sysIdDynamicForward() {
		if (routine == null) return Commands.none();
		return routine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command sysIdDynamicReverse() {
		if (routine == null) return Commands.none();
		return routine.dynamic(SysIdRoutine.Direction.kReverse);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Collector", inputs);
	}
}
