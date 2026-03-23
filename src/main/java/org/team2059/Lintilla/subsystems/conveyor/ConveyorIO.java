package org.team2059.Lintilla.subsystems.conveyor;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ConveyorIO {
	default public void setConveyorSpeed(double speed) {}

	default public void stopConveyor() {}

	default public void updateInputs(ConveyorIOInputs inputs) {}

	@AutoLog
	class ConveyorIOInputs {
		public MutTemperature conveyorTemp = Celsius.mutable(0);
		public MutVoltage conveyorAppliedVolts = Volts.mutable(0);
		public MutCurrent conveyorCurrent = Amps.mutable(0);
	}
}
