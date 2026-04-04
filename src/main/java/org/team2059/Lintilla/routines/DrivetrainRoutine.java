package org.team2059.Lintilla.routines;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static org.team2059.Lintilla.Constants.OperatorConstants.tuningMode;

public class DrivetrainRoutine {

	private final SysIdRoutine sysIdRoutine;

	// Mutable holders for unit-safe voltage, linear distance, and linear velocity values, persisted to avoid reallocation.
	private final MutVoltage appliedVolts;
	private final MutDistance distance;
	private final MutLinearVelocity velocity;

	public DrivetrainRoutine(Drivetrain drivetrain) {
		if (tuningMode) {
			appliedVolts = Volts.mutable(0);
			distance = Meters.mutable(0);
			velocity = MetersPerSecond.mutable(0);

			// SysID characterization configuration
			sysIdRoutine = new SysIdRoutine(
			  new SysIdRoutine.Config(
				Volts.of(1).per(Units.Second), // Ramp rate in V/s
				Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
				Time.ofBaseUnits(10, Units.Second), // Use default timeout of 10 sec
				null
			  ),
			  new SysIdRoutine.Mechanism(
				// Tell SysID how to plumb the driving voltage to the motors
				voltage -> {
					double volts = voltage.in(Volts);
					for (int i = 0; i < 4; i++) {
						drivetrain.runCharacterization(volts);
					}
				},
				// Tell SysID how to record a frame of data for each motor on the mechanism
				log -> {
					log.motor("drive")
					  .voltage(appliedVolts.mut_replace(drivetrain.getSwerveInputs()[0].driveAppliedVolts))
					  .linearPosition(distance.mut_replace(drivetrain.getSwerveInputs()[0].drivePos))
					  .linearVelocity(velocity.mut_replace(drivetrain.getSwerveInputs()[0].driveVel));
				},
				// Tell SysId to make generated commands require this subsystem, suffix test state in
				// WPILog with this subsystem's name
				drivetrain
			  )
			);
		} else {
			sysIdRoutine = null;
			appliedVolts = null;
			distance = null;
			velocity = null;
		}
	}

	// Returns a command that will execute a quasistatic test in the given direction
	public Command quasistaticForward() {
		if (sysIdRoutine == null) return Commands.none();
		return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
	}

	public Command quasistaticReverse() {
		if (sysIdRoutine == null) return Commands.none();
		return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
	}

	// Returns a command that will execute a dynamic test in the given direction
	public Command dynamicForward() {
		if (sysIdRoutine == null) return Commands.none();
		return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
	}

	public Command dynamicReverse() {
		if (sysIdRoutine == null) return Commands.none();
		return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
	}
}
