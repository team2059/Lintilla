package org.team2059.Lintilla.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ShooterBase extends SubsystemBase {

  private final ShooterIO leftShooter;
  private final ShooterIO rightShooter;

  private final ShooterIOInputsAutoLogged leftShooterInputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOInputsAutoLogged rightShooterInputs = new ShooterIOInputsAutoLogged();

  private final SparkFlex indexerMotor;
  private final SparkFlexConfig indexerMotorConfig = new SparkFlexConfig();

  private final SysIdRoutine leftRoutine;
  private final SysIdRoutine rightRoutine;

  // Variables below are used exclusively for SysID routine logging.
  private final MutVoltage leftAppliedVoltsRoutine = Volts.mutable(0);
  private final MutAngle leftAngleRoutine = Rotations.mutable(0);
  private final MutAngularVelocity leftAngularVelocityRoutine = RPM.mutable(0);
  private final MutVoltage rightAppliedVoltsRoutine = Volts.mutable(0);
  private final MutAngle rightAngleRoutine = Rotations.mutable(0);
  private final MutAngularVelocity rightAngularVelocityRoutine = RPM.mutable(0);

  public ShooterBase(
    ShooterIO leftShooter,
    ShooterIO rightShooter,
    int indexerMotorCanId
  ) {
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;

    // Configure indexer motor (account for prototypes which have no indexer)
    if (indexerMotorCanId != -1) {
        indexerMotor = new SparkFlex(indexerMotorCanId, SparkLowLevel.MotorType.kBrushless);
        indexerMotorConfig
          .inverted(false)
          .idleMode(SparkBaseConfig.IdleMode.kBrake);
        indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexerMotor.clearFaults();
    } else {
        indexerMotor = null;
    }

    leftRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        voltage -> {
          leftShooter.setFlywheelVoltage(voltage.in(Volts));
        },
        log -> {
          log.motor("left-shooter-motor")
            .voltage(leftAppliedVoltsRoutine.mut_replace(leftShooterInputs.flywheelAppliedVolts))
            .angularPosition(leftAngleRoutine.mut_replace(leftShooterInputs.flywheelPosition))
            .angularVelocity(leftAngularVelocityRoutine.mut_replace(leftShooterInputs.flywheelVelocity));
        },
        this
      )
    );

    rightRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        voltage -> {
          rightShooter.setFlywheelVoltage(voltage.in(Volts));
        },
        log -> {
          log.motor("right-shooter-motor")
            .voltage(rightAppliedVoltsRoutine.mut_replace(rightShooterInputs.flywheelAppliedVolts))
            .angularPosition(rightAngleRoutine.mut_replace(rightShooterInputs.flywheelPosition))
            .angularVelocity(rightAngularVelocityRoutine.mut_replace(rightShooterInputs.flywheelVelocity));
        },
        this
      )
    );
  }

  public void setLeftShooterVoltage(double volts) {
      leftShooter.setFlywheelVoltage(volts);
  }

  public void setRightShooterVoltage(double volts) {
      leftShooter.setFlywheelVoltage(volts);
  }

  public void stopLeftShooter() {
      leftShooter.stopFlywheel();
  }

  public void stopRightShooter() {
      rightShooter.stopFlywheel();
  }

  public void stopBothShooters() {
      stopLeftShooter();
      stopRightShooter();
  }

  public Command leftSysIdQuasistaticForward() { return leftRoutine.quasistatic(SysIdRoutine.Direction.kForward); }

  public Command leftSysIdQuasistaticReverse() { return leftRoutine.quasistatic(SysIdRoutine.Direction.kReverse); }

  public Command rightSysIdQuasistaticForward() { return rightRoutine.quasistatic(SysIdRoutine.Direction.kForward); }

  public Command rightSysIdQuasistaticReverse() { return rightRoutine.quasistatic(SysIdRoutine.Direction.kReverse); }

  public Command leftSysIdDynamicForward() { return leftRoutine.dynamic(SysIdRoutine.Direction.kForward); }

  public Command leftSysIdDynamicReverse() { return leftRoutine.dynamic(SysIdRoutine.Direction.kReverse); }

  public Command rightSysIdDynamicForward() { return rightRoutine.dynamic(SysIdRoutine.Direction.kForward); }

  public Command rightSysIdDynamicReverse() { return rightRoutine.dynamic(SysIdRoutine.Direction.kReverse); }

  @Override
  public void periodic() {
    leftShooter.updateInputs(leftShooterInputs);
    rightShooter.updateInputs(rightShooterInputs);

    Logger.processInputs("ShooterBase/Left", leftShooterInputs);
    Logger.processInputs("ShooterBase/Right", rightShooterInputs);
  }
}
