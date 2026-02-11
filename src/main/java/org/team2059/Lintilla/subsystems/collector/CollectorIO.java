// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla.subsystems.collector;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface CollectorIO {
  @AutoLog
  class CollectorIOInputs {
    public MutTemperature intakeTemp = Celsius.mutable(0);
    public MutVoltage intakeAppliedVolts = Volts.mutable(0);
    public MutCurrent intakeCurrent = Amps.mutable(0);

    public MutTemperature conveyorTemp = Celsius.mutable(0);
    public MutVoltage conveyorAppliedVolts = Volts.mutable(0);
    public MutCurrent conveyorCurrent = Amps.mutable(0);

    public MutTemperature tiltTemp = Celsius.mutable(0);
    public MutVoltage tiltAppliedVolts = Volts.mutable(0);
    public MutCurrent tiltCurrent = Amps.mutable(0);
    public MutAngle tiltPosition = Rotations.mutable(0);
    public MutAngularVelocity tiltVelocity = RPM.mutable(0);
  }

  default public void setIntakeSpeed(double speed) {}

  default public void stopCollector() {}

  default public void setTiltSpeed(double speed) {}

  default public void setTiltVolts(double volts) {}

  default public void stopTilt() {}

  default public void setTiltPosition(double position) {}

  default public void runConveyor(double speed) {}

  default public void stopConveyor() {}

  default public void updateInputs(CollectorIOInputs inputs) {}
}