package org.team2059.Lintilla.routines;

import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;

import static edu.wpi.first.units.Units.*;


import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DrivetrainRoutine {

    private final SysIdRoutine sysIdRoutine;

    // Mutable holders for unit-safe voltage, linear distance, and linear velocity values, persisted to avoid reallocation.
    private final MutVoltage driveRoutineAppliedVoltage = Volts.mutable(0);
    private final MutDistance driveRoutineDistance = Meters.mutable(0);
    private final MutLinearVelocity driveRoutineVelocity = MetersPerSecond.mutable(0);

    public DrivetrainRoutine(Drivetrain drivetrain) {
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
                    drivetrain.setModulesToZeroRadPID();
                    drivetrain.getFrontLeft().setDriveVoltage(voltage.in(Volts));
                    drivetrain.getFrontRight().setDriveVoltage(voltage.in(Volts));
                    drivetrain.getBackLeft().setDriveVoltage(voltage.in(Volts));
                    drivetrain.getBackRight().setDriveVoltage(voltage.in(Volts));
                }, 
                // Tell SysID how to record a frame of data for each motor on the mechanism
                log -> {
                    log.motor("frontleft")
                        .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.getSwerveInputs()[0].driveAppliedVolts))
                        .linearPosition(driveRoutineDistance.mut_replace(drivetrain.getSwerveInputs()[0].drivePos))
                        .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.getSwerveInputs()[0].driveVel));

                    log.motor("frontright")
                        .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.getSwerveInputs()[1].driveAppliedVolts))
                        .linearPosition(driveRoutineDistance.mut_replace(drivetrain.getSwerveInputs()[1].drivePos))
                        .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.getSwerveInputs()[1].driveVel));

                    log.motor("backleft")
                        .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.getSwerveInputs()[2].driveAppliedVolts))
                        .linearPosition(driveRoutineDistance.mut_replace(drivetrain.getSwerveInputs()[2].drivePos))
                        .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.getSwerveInputs()[2].driveVel));

                    log.motor("backright")
                        .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.getSwerveInputs()[3].driveAppliedVolts))
                        .linearPosition(driveRoutineDistance.mut_replace(drivetrain.getSwerveInputs()[3].drivePos))
                        .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.getSwerveInputs()[3].driveVel));
                }, 
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name
                drivetrain
            )
        );
    }

    // Returns a command that will execute a quasistatic test in the given direction
    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }
    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    // Returns a command that will execute a dynamic test in the given direction
    public Command dynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }
    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
