package org.team2059.Lintilla.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngle;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Rotations;

public interface GyroscopeIO {
	default public void updateInputs(GyroscopeIOInputs inputs) {}

	default public Rotation2d getRotation2d() {return Rotation2d.kZero;}

	;

	default public void reset() {}

	@AutoLog
	class GyroscopeIOInputs {
		public MutAngle roll = Rotations.mutable(0);
		public MutAngle yaw = Rotations.mutable(0);
		public MutAngle pitch = Rotations.mutable(0);
	}

	;
}