// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla.subsystems.collector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.Constants.CollectorConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CollectorIOReal implements CollectorIO {

  public SparkFlex tiltMotor;
  public SparkFlex intakeMotor;

  public SparkClosedLoopController tiltController;

  public static SparkFlexConfig tiltConfig = new SparkFlexConfig();
  public SparkFlexConfig intakeConfig = new SparkFlexConfig();

  public AbsoluteEncoder thruBoreEnc;

  public CollectorIOReal(SparkFlex tiltMotor, SparkFlex intakeMotor) {
    this.intakeMotor = intakeMotor;
    this.tiltMotor = tiltMotor;

    tiltConfig
        .inverted(false)
        .idleMode(SparkFlexConfig.IdleMode.kBrake);

    tiltConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(CollectorConstants.kPTilt, 0.0, 0.0)
        .outputRange(-1, 1);

    tiltConfig.absoluteEncoder
        .zeroOffset(CollectorConstants.thruBoreOffset)
        .inverted(true);

    tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tiltMotor.clearFaults();

    thruBoreEnc = tiltMotor.getAbsoluteEncoder();
    SmartDashboard.putNumber("ThruBorePos", thruBoreEnc.getPosition());

    intakeConfig
        .inverted(false)
        .idleMode(SparkFlexConfig.IdleMode.kBrake);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.clearFaults();

    tiltController = tiltMotor.getClosedLoopController();
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void stopCollector() {
    intakeMotor.set(0);
  }

  @Override
  public void setTiltSpeed(double speed) {
    tiltMotor.set(speed);
  }

  @Override
  public void setTiltPosition(double position) {
    // tiltController.setSetpoint(MathUtil.clamp(position,
    // CollectorConstants.thruBoreIn, CollectorConstants.thruBoreOut),
    // ControlType.kPosition);
    tiltController.setSetpoint(MathUtil.clamp(position, CollectorConstants.thruBoreOut, CollectorConstants.thruBoreIn),
        ControlType.kPosition);
    Logger.recordOutput("Setpoint",
        tiltController.getSetpoint());
    
  }

  @Override
  public void setTiltVolts(double volts) {
    tiltMotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void stopTilt() {
    tiltMotor.set(0);
  }

  @Override
  public void updateInputs(CollectorIOInputs inputs) {
    inputs.intakeAppliedVolts.mut_replace(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage(), Volts);
    inputs.intakeCurrent.mut_replace(intakeMotor.getOutputCurrent(), Amps);
    inputs.intakeTemp.mut_replace(intakeMotor.getMotorTemperature(), Celsius);
    inputs.tiltAppliedVolts.mut_replace(tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage(), Volts);
    inputs.tiltCurrent.mut_replace(tiltMotor.getOutputCurrent(), Amps);
    inputs.tiltTemp.mut_replace(tiltMotor.getMotorTemperature(), Celsius);
    inputs.tiltPosition.mut_replace(thruBoreEnc.getPosition(), Rotations);
    inputs.tiltVelocity.mut_replace(thruBoreEnc.getVelocity(), RPM);
  }
}
