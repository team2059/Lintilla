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
	private final MutVoltage flAppliedVolts;
	private final MutDistance flDistance;
	private final MutLinearVelocity flVelocity;

	private final MutVoltage frAppliedVolts;
	private final MutDistance frDistance;
	private final MutLinearVelocity frVelocity;

	private final MutVoltage blAppliedVolts;
	private final MutDistance blDistance;
	private final MutLinearVelocity blVelocity;

	private final MutVoltage brAppliedVolts;
	private final MutDistance brDistance;
	private final MutLinearVelocity brVelocity;

	public DrivetrainRoutine(Drivetrain drivetrain) {
		if (tuningMode) {
			flAppliedVolts = Volts.mutable(0);
			flDistance = Meters.mutable(0);
			flVelocity = MetersPerSecond.mutable(0);

			frAppliedVolts = Volts.mutable(0);
			frDistance = Meters.mutable(0);
			frVelocity = MetersPerSecond.mutable(0);

			blAppliedVolts = Volts.mutable(0);
			blDistance = Meters.mutable(0);
			blVelocity = MetersPerSecond.mutable(0);

			brAppliedVolts = Volts.mutable(0);
			brDistance = Meters.mutable(0);
			brVelocity = MetersPerSecond.mutable(0);

			// SysID characterization configuration
			sysIdRoutine = new SysIdRoutine(
			  new SysIdRoutine.Config(
				Volts.of(1).per(Units.Second), // Ramp rate in V/s
				Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
				Time.ofBaseUnits(7, Units.Second), // Use default timeout of 10 sec
				null
			  ),
			  new SysIdRoutine.Mechanism(
				// Tell SysID how to plumb the driving voltage to the motors
				voltage -> {
					drivetrain.getFrontLeft().setDriveVoltage(voltage.in(Volts));
					drivetrain.getFrontRight().setDriveVoltage(voltage.in(Volts));
					drivetrain.getBackLeft().setDriveVoltage(voltage.in(Volts));
					drivetrain.getBackRight().setDriveVoltage(voltage.in(Volts));
				},
				// Tell SysID how to record a frame of data for each motor on the mechanism
				log -> {
					log.motor("frontleft")
					  .voltage(flAppliedVolts.mut_replace(drivetrain.getSwerveInputs()[0].driveAppliedVolts))
					  .linearPosition(flDistance.mut_replace(drivetrain.getSwerveInputs()[0].drivePos))
					  .linearVelocity(flVelocity.mut_replace(drivetrain.getSwerveInputs()[0].driveVel));

					log.motor("frontright")
					  .voltage(frAppliedVolts.mut_replace(drivetrain.getSwerveInputs()[1].driveAppliedVolts))
					  .linearPosition(frDistance.mut_replace(drivetrain.getSwerveInputs()[1].drivePos))
					  .linearVelocity(frVelocity.mut_replace(drivetrain.getSwerveInputs()[1].driveVel));

					log.motor("backleft")
					  .voltage(blAppliedVolts.mut_replace(drivetrain.getSwerveInputs()[2].driveAppliedVolts))
					  .linearPosition(blDistance.mut_replace(drivetrain.getSwerveInputs()[2].drivePos))
					  .linearVelocity(blVelocity.mut_replace(drivetrain.getSwerveInputs()[2].driveVel));

					log.motor("backright")
					  .voltage(brAppliedVolts.mut_replace(drivetrain.getSwerveInputs()[3].driveAppliedVolts))
					  .linearPosition(brDistance.mut_replace(drivetrain.getSwerveInputs()[3].drivePos))
					  .linearVelocity(brVelocity.mut_replace(drivetrain.getSwerveInputs()[3].driveVel));
				},
				// Tell SysId to make generated commands require this subsystem, suffix test state in
				// WPILog with this subsystem's name
				drivetrain
			  )
			);
		} else {
			sysIdRoutine = null;
			flAppliedVolts = null;
			frAppliedVolts = null;
			blAppliedVolts = null;
			brAppliedVolts = null;
			flDistance = null;
			frDistance = null;
			blDistance = null;
			brDistance = null;
			flVelocity = null;
			frVelocity = null;
			blVelocity = null;
			brVelocity = null;
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
