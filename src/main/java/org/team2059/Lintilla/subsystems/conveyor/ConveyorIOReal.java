package org.team2059.Lintilla.subsystems.conveyor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.*;
import static org.team2059.Lintilla.Constants.CANConstants.*;

public class ConveyorIOReal implements ConveyorIO {
	private final SparkFlex conveyorMotor;
	private final SparkFlexConfig conveyorConfig = new SparkFlexConfig();

	public ConveyorIOReal(int conveyorMotorCanId) {
		conveyorMotor = new SparkFlex(conveyorMotorCanId, SparkLowLevel.MotorType.kBrushless);

		conveyorConfig
		  .inverted(false)
		  .idleMode(SparkFlexConfig.IdleMode.kBrake);

		conveyorConfig.signals
		  .primaryEncoderPositionPeriodMs(REV_POSITION_PERIOD_MS)
		  .primaryEncoderVelocityPeriodMs(REV_VELOCITY_PERIOD_MS)
		  .appliedOutputPeriodMs(REV_APPLIED_OUTPUT_PERIOD_MS)
		  .outputCurrentPeriodMs(REV_OUTPUT_CURRENT_PERIOD_MS)
		  .motorTemperaturePeriodMs(REV_MOTOR_TEMP_PERIOD_MS)
		  .faultsPeriodMs(REV_MOTOR_FAULTS_PERIOD_MS);

		conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		conveyorMotor.clearFaults();
	}

	@Override
	public void setConveyorSpeed(double speed) {
		conveyorMotor.set(speed);
	}

	@Override
	public void stopConveyor() {
		conveyorMotor.setVoltage(0);
	}

	@Override
	public void updateInputs(ConveyorIOInputs inputs) {
		inputs.conveyorAppliedVolts.mut_replace(conveyorMotor.getAppliedOutput() * conveyorMotor.getBusVoltage(), Volts);
		inputs.conveyorCurrent.mut_replace(conveyorMotor.getOutputCurrent(), Amps);
		inputs.conveyorTemp.mut_replace(conveyorMotor.getMotorTemperature(), Celsius);
	}

}
