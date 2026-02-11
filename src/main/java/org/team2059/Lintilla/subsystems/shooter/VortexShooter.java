package org.team2059.Lintilla.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.*;

public class VortexShooter implements ShooterIO {

	private final SparkFlex flywheelMotor;
	private final SparkFlexConfig flywheelMotorConfig = new SparkFlexConfig();

	private final SparkClosedLoopController closedLoopController;

	private final RelativeEncoder encoder;

	public VortexShooter(
	  int flywheelMotorCanId,
	  boolean flywheelMotorInverted,
	  double kP, double kI, double kD,
	  double kS, double kV, double kA
	) {

		flywheelMotor = new SparkFlex(flywheelMotorCanId, SparkLowLevel.MotorType.kBrushless);

		flywheelMotorConfig
		  .inverted(flywheelMotorInverted)
		  .idleMode(SparkBaseConfig.IdleMode.kCoast);

		// Set initial closed-loop gains
		flywheelMotorConfig.closedLoop
		  .pid(kP, kI, kD)
		  .feedForward
		  .kS(kS).kV(kV).kA(kA);

		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		flywheelMotor.clearFaults();

		closedLoopController = flywheelMotor.getClosedLoopController();

		encoder = flywheelMotor.getEncoder();
	}

	@Override
	public void setFlywheelkP(double kP) {
		flywheelMotorConfig.closedLoop.p(kP);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkI(double kI) {
		flywheelMotorConfig.closedLoop.i(kI);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkD(double kD) {
		flywheelMotorConfig.closedLoop.d(kD);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkS(double kS) {
		flywheelMotorConfig.closedLoop.feedForward.kS(kS);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkV(double kV) {
		flywheelMotorConfig.closedLoop.feedForward.kV(kV);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkA(double kA) {
		flywheelMotorConfig.closedLoop.feedForward.kA(kA);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelVoltage(double volts) {
		flywheelMotor.setVoltage(volts);
	}

	@Override
	public void setFlywheelRpm(double rpm) {
		closedLoopController.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
	}

	@Override
	public void stopFlywheel() {
		flywheelMotor.stopMotor();
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.flywheelPosition.mut_replace(encoder.getPosition(), Rotations);
		inputs.flywheelVelocity.mut_replace(encoder.getVelocity(), RPM);
		inputs.flywheelAppliedVolts.mut_replace(flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage(), Volts);
		inputs.flywheelCurrent.mut_replace(flywheelMotor.getOutputCurrent(), Amps);
		inputs.flywheelTemp.mut_replace(flywheelMotor.getMotorTemperature(), Celsius);
	}
}