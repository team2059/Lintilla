package org.team2059.Lintilla.subsystems.shooter;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

// Drum is right motor and follower is left motor
public interface ShooterIO {

    default void setDrumkP(double kP) {}

    default void setDrumkI(double kI) {}

    default void setDrumkD(double kD) {}

    default void setDrumkS(double kS) {}

    default void setDrumkV(double kV) {}

    default void setDrumkA(double kA) {}

    default void setDrumVoltage(double volts) {}

    default void setDrumRpm(double rpm) {}

    default void setDrumSpeed(double speed) {}

    default void setRightMotorSpeed(double speed) {}

    default void stopDrum() {}

    default void setIndexerkP(double kP) {}

    default void setIndexerkI(double kI) {}

    default void setIndexerkD(double kD) {}

    default void setIndexerkS(double kS) {}

    default void setIndexerkV(double kV) {}

    default void setIndexerkA(double kA) {}

    default void setIndexerVoltage(double volts) {}

    default void setIndexerRpm(double rpm) {}

    default void setIndexerSpeed(double speed) {}

    default void stopIndexer() {}

    default void updateInputs(ShooterIOInputs inputs) {}


    @AutoLog
    class ShooterIOInputs {

    public MutAngle drumPosition = Rotations.mutable(0);
    public MutAngularVelocity drumVelocity = RPM.mutable(0);

    public MutVoltage drumAppliedVolts = Volts.mutable(0);
    public MutCurrent drumCurrent = Amps.mutable(0);
    public MutTemperature drumTemp = Celsius.mutable(0);

    public MutCurrent drumFollowerCurrent = Amps.mutable(0);
    public MutTemperature drumFollowerTemp = Celsius.mutable(0);

    public MutAngle indexerPosition = Rotations.mutable(0);
    public MutAngularVelocity indexerVelocity = RPM.mutable(0);
    public MutVoltage indexerAppliedVolts = Volts.mutable(0);
    public MutCurrent indexerCurrent = Amps.mutable(0);
    public MutTemperature indexerTemp = Celsius.mutable(0);

    public MutCurrent indexerFollowerCurrent = Amps.mutable(0);
    public MutTemperature indexerFollowerTemp = Celsius.mutable(0);
}
}
