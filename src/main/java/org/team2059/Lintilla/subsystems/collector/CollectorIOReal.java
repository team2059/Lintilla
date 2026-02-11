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
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;
import org.team2059.Lintilla.util.LoggedTunableNumber;
import org.team2059.Lintilla.Constants.CollectorConstants;

import static edu.wpi.first.units.Units.*;

public class CollectorIOReal implements CollectorIO {

	public SparkFlex tiltMotor;
	public SparkFlex intakeMotor;
	public SparkFlex conveyorMotor;

	public SparkClosedLoopController tiltController;

	public SparkFlexConfig tiltConfig = new SparkFlexConfig();
	public SparkFlexConfig intakeConfig = new SparkFlexConfig();
	public SparkFlexConfig conveyorConfig = new SparkFlexConfig();
	public AbsoluteEncoder thruBoreEnc;

	public CollectorIOReal(int tiltMotorCanId, int intakeMotorCanId, int conveyorMotorCanId) {
		intakeMotor = new SparkFlex(intakeMotorCanId, SparkLowLevel.MotorType.kBrushless);
		tiltMotor = new SparkFlex(tiltMotorCanId, SparkLowLevel.MotorType.kBrushless);
		conveyorMotor = new SparkFlex(conveyorMotorCanId, SparkLowLevel.MotorType.kBrushless);

		tiltConfig
		  .inverted(false)
		  .idleMode(SparkFlexConfig.IdleMode.kBrake);

		tiltConfig.closedLoop
		  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
		  .pid(CollectorConstants.kPTilt, CollectorConstants.kITilt, CollectorConstants.kDTilt)
		  .outputRange(-1, 1);

		tiltConfig.closedLoop.feedForward
		  .kCos(CollectorConstants.kCosTilt)
		  .kS(CollectorConstants.kSTilt)
		  .kV(CollectorConstants.kVTilt)
		  .kA(CollectorConstants.kATilt);

		tiltConfig.absoluteEncoder
		  .inverted(true)
		  .zeroCentered(true)
		  .zeroOffset(CollectorConstants.thruBoreOffset);

		tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		tiltMotor.clearFaults();

		thruBoreEnc = tiltMotor.getAbsoluteEncoder();

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
		intakeMotor.set(0);
	}

	@Override
	public void setTiltSpeed(double speed) {
		tiltMotor.set(MathUtil.clamp(speed, -0.3, 0.3));
	}

	@Override
	public void setTiltPosition(double position) {
		tiltController.setSetpoint(position, ControlType.kPosition);
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
	public void runConveyor(double speed) {
		conveyorMotor.set(speed);
	}

	@Override
	public void stopConveyor() {
		conveyorMotor.set(0);
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

		inputs.conveyorAppliedVolts.mut_replace(conveyorMotor.getAppliedOutput() * conveyorMotor.getBusVoltage(), Volts);
		inputs.conveyorCurrent.mut_replace(conveyorMotor.getOutputCurrent(), Amps);
		inputs.conveyorTemp.mut_replace(conveyorMotor.getMotorTemperature(), Celsius);
	}
}
