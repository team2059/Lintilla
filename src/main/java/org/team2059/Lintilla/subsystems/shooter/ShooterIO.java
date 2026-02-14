package org.team2059.Lintilla.subsystems.shooter;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {
	default public void setFlywheelkP(double kP) {}
	default public void setFlywheelkI(double kI) {}
	default public void setFlywheelkD(double kD) {}
	default public void setFlywheelkS(double kS) {}
	default public void setFlywheelkV(double kV) {}
	default public void setFlywheelkA(double kA) {}

	default public void setIndexerkP(double kP) {}
	default public void setIndexerkI(double kI) {}
	default public void setIndexerkD(double kD) {}
	default public void setIndexerkS(double kS) {}
	default public void setIndexerkV(double kV) {}
	default public void setIndexerkA(double kA) {}

	default public void setFlywheelVoltage(double volts) {}
	default public void setIndexerVoltage(double volts) {}

	default public void setFlywheelRpm(double rpm) {}
	default public void setIndexerRpm(double rpm) {}

	default public void stopFlywheel() {}
	default public void stopIndexer() {}

	default public void setIndexerSpeed(double speed) {}

	default public void updateInputs(ShooterIOInputs inputs) {}


	@AutoLog
	class ShooterIOInputs {
		public MutAngle flywheelPosition = Rotations.mutable(0);
		public MutAngle indexerPosition = Rotations.mutable(0);

		public MutAngularVelocity flywheelVelocity = RPM.mutable(0);
		public MutAngularVelocity indexerVelocity = RPM.mutable(0);

		public MutVoltage flywheelAppliedVolts = Volts.mutable(0);
		public MutVoltage indexerAppliedVolts = Volts.mutable(0);

		public MutCurrent flywheelCurrent = Amps.mutable(0);
		public MutCurrent indexerCurrent = Amps.mutable(0);

		public MutTemperature flywheelTemp = Celsius.mutable(0);
		public MutTemperature indexerTemp = Celsius.mutable(0);
	}
}
