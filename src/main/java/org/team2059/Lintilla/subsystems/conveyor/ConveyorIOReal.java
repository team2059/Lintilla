package org.team2059.Lintilla.subsystems.conveyor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.*;

public class ConveyorIOReal implements ConveyorIO {
	private final SparkFlex conveyorMotor;
	private final SparkFlexConfig conveyorConfig = new SparkFlexConfig();

	public ConveyorIOReal(int conveyorMotorCanId) {
		conveyorMotor = new SparkFlex(conveyorMotorCanId, SparkLowLevel.MotorType.kBrushless);

		conveyorConfig
		  .inverted(false)
		  .idleMode(SparkFlexConfig.IdleMode.kBrake);

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
