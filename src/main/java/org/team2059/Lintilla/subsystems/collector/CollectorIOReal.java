// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla.subsystems.collector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.team2059.Lintilla.Constants.CollectorConstants;
import org.team2059.Lintilla.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public class CollectorIOReal implements CollectorIO {

	private final SparkFlex tiltMotor;
	private final SparkFlex intakeMotor;
	private final SparkFlex conveyorMotor;

	private final SparkClosedLoopController tiltController;

	private final SparkFlexConfig tiltConfig = new SparkFlexConfig();
	private final SparkFlexConfig intakeConfig = new SparkFlexConfig();
	private final SparkFlexConfig conveyorConfig = new SparkFlexConfig();
	private final AbsoluteEncoder thruBoreEnc;

	private final LoggedTunableNumber kPTilt = new LoggedTunableNumber("kPTilt", CollectorConstants.kPTilt);

	public CollectorIOReal(int tiltMotorCanId, int intakeMotorCanId, int conveyorMotorCanId) {
		// Initialize motors
		intakeMotor = new SparkFlex(intakeMotorCanId, SparkLowLevel.MotorType.kBrushless);
		tiltMotor = new SparkFlex(tiltMotorCanId, SparkLowLevel.MotorType.kBrushless);
		conveyorMotor = new SparkFlex(conveyorMotorCanId, SparkLowLevel.MotorType.kBrushless);

		// Configure tilt motor - this is the most complicated part of this entire subsystem
		tiltConfig
		  .inverted(false)
		  .idleMode(SparkFlexConfig.IdleMode.kBrake);

		tiltConfig.closedLoop
		  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // takes thrubore connected thru encoder port
		  .pid(kPTilt.get(), CollectorConstants.kITilt, CollectorConstants.kDTilt)
		  .outputRange(-1, 1); // can be adjusted if we're being too harsh

		// NEEDED because of shooter mass & torque requirements from gravity. ArmFeedforward is WPILib alternative
		tiltConfig.closedLoop.feedForward
		  .kCos(CollectorConstants.kCosTilt)
		  .kS(CollectorConstants.kSTilt)
		  .kV(CollectorConstants.kVTilt)
		  .kA(CollectorConstants.kATilt);

		// Makes reported rotations [-0.25, 0.25]
		tiltConfig.absoluteEncoder
		  .inverted(true)
		  .zeroCentered(true)
		  .zeroOffset(CollectorConstants.thruBoreOffset);

		// Push the config to the tilt motor
		tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		tiltMotor.clearFaults();

		thruBoreEnc = tiltMotor.getAbsoluteEncoder();

		// Configure intake and conveyor, simple bang-bang
		intakeConfig
		  .inverted(false)
		  .idleMode(SparkFlexConfig.IdleMode.kBrake);
		intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		intakeMotor.clearFaults();

		conveyorConfig
		  .inverted(false)
		  .idleMode(SparkFlexConfig.IdleMode.kBrake);

		conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		conveyorMotor.clearFaults();

		tiltController = tiltMotor.getClosedLoopController();
	}

	@Override
	public void setIntakeSpeed(double speed) {
		intakeMotor.set(speed);
	}

	@Override
	public void stopCollector() {
		intakeMotor.setVoltage(0);
	}

	@Override
	public void setTiltSpeed(double speed) {
		tiltMotor.set(speed);
	}

	@Override
	public void setTiltPosition(double position) {
		tiltController.setSetpoint(position, ControlType.kPosition);
	}

	@Override
	public void setTiltVolts(double volts) {
		tiltMotor.setVoltage(volts);
	}

	@Override
	public void stopTilt() {
		tiltMotor.setVoltage(0);
	}

	@Override
	public void runConveyor(double speed) {
		conveyorMotor.set(speed);
	}

	@Override
	public void stopConveyor() {
		conveyorMotor.setVoltage(0);
	}

	@Override
	public void updateInputs(CollectorIOInputs inputs) {

		LoggedTunableNumber.ifChanged(
		  hashCode(),
		  () -> {
			  tiltConfig.closedLoop.pid(kPTilt.get(), 0, 0);
			  tiltMotor.configure(tiltConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		  },
		  kPTilt
		);

		inputs.intakeAppliedVolts.mut_replace(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage(), Volts);
		inputs.intakeCurrent.mut_replace(intakeMotor.getOutputCurrent(), Amps);
		inputs.intakeTemp.mut_replace(intakeMotor.getMotorTemperature(), Celsius);

		inputs.tiltAppliedVolts.mut_replace(tiltMotor.getAppliedOutput() * tiltMotor.getBusVoltage(), Volts);
		inputs.tiltCurrent.mut_replace(tiltMotor.getOutputCurrent(), Amps);
		inputs.tiltTemp.mut_replace(tiltMotor.getMotorTemperature(), Celsius);
		inputs.tiltPosition.mut_replace(thruBoreEnc.getPosition(), Rotations);
		inputs.tiltVelocity.mut_replace(thruBoreEnc.getVelocity(), RPM);

		inputs.conveyorAppliedVolts.mut_replace(conveyorMotor.getAppliedOutput() * conveyorMotor.getBusVoltage(), Volts);
		inputs.conveyorCurrent.mut_replace(conveyorMotor.getOutputCurrent(), Amps);
		inputs.conveyorTemp.mut_replace(conveyorMotor.getMotorTemperature(), Celsius);
	}
}
