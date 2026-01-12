package org.team2059.Lintilla.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface SwerveModuleIO {
  @AutoLog
  class SwerveModuleIOInputs {
    public MutDistance drivePos = Meters.mutable(0);
    public MutLinearVelocity driveVel = MetersPerSecond.mutable(0);
    public MutVoltage driveAppliedVolts = Volts.mutable(0);
    public MutCurrent driveCurrent = Amps.mutable(0);
    public MutTemperature driveTemp = Celsius.mutable(0);

    public MutAngle azimuthAbsolutePos = Rotations.mutable(0);
    public MutAngularVelocity azimuthVel = RotationsPerSecond.mutable(0);
    public MutVoltage azimuthAppliedVolts = Volts.mutable(0);
    public MutCurrent azimuthCurrent = Amps.mutable(0);
    public MutTemperature azimuthTemp = Celsius.mutable(0);
  }

  default public void updateInputs(SwerveModuleIOInputs inputs) {};

  default public void resetEncoders() {};

  default public void setState(SwerveModuleState desiredState) {};

  default public void stop() {};

  default public void setDriveVoltage(double volts) {};

  default public void setAzimuthVoltage(double volts) {};

  default public SwerveModuleState getState() {return new SwerveModuleState();}
}