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
	private final SparkFlex indexerMotor;

	private final SparkFlexConfig flywheelMotorConfig = new SparkFlexConfig();
	private final SparkFlexConfig indexerMotorConfig = new SparkFlexConfig();

	private final SparkClosedLoopController flywheelController;
	private final SparkClosedLoopController indexerController;

	private final RelativeEncoder flywheelEncoder;
	private final RelativeEncoder indexerEncoder;

	public VortexShooter(
	  int flywheelMotorCanId,
	  int indexerMotorCanId,
	  boolean flywheelMotorInverted,
	  boolean indexerMotorInverted,
	  double kPFlywheel, double kIFlywheel, double kDFlywheel, double kSFlywheel, double kVFlywheel, double kAFlywheel,
	  double kPIndexer, double kIIndexer, double kDIndexer, double kSIndexer, double kVIndexer, double kAIndexer
	) {

		flywheelMotor = new SparkFlex(flywheelMotorCanId, SparkLowLevel.MotorType.kBrushless);
		indexerMotor = new SparkFlex(indexerMotorCanId, SparkLowLevel.MotorType.kBrushless);

		flywheelMotorConfig
		  .inverted(flywheelMotorInverted)
		  .idleMode(SparkBaseConfig.IdleMode.kCoast);
		indexerMotorConfig
		  .inverted(indexerMotorInverted)
		  .idleMode(SparkBaseConfig.IdleMode.kBrake);

		// Set initial closed-loop gains
		flywheelMotorConfig.closedLoop
		  .pid(kPFlywheel, kIFlywheel, kDFlywheel)
		  .feedForward
		  .kS(kSFlywheel).kV(kVFlywheel).kA(kAFlywheel);
		indexerMotorConfig.closedLoop
		  .pid(kPIndexer, kIIndexer, kDIndexer)
		  .feedForward
		  .kS(kSIndexer).kV(kVIndexer).kA(kAIndexer);

		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		flywheelMotor.clearFaults();
		indexerMotor.clearFaults();

		flywheelController = flywheelMotor.getClosedLoopController();
		indexerController = indexerMotor.getClosedLoopController();

		flywheelEncoder = flywheelMotor.getEncoder();
		indexerEncoder = indexerMotor.getEncoder();
	}

	@Override
	public void setFlywheelkP(double kP) {
		flywheelMotorConfig.closedLoop.p(kP);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setIndexerkP(double kP) {
		indexerMotorConfig.closedLoop.p(kP);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkI(double kI) {
		flywheelMotorConfig.closedLoop.i(kI);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setIndexerkI(double kI) {
		indexerMotorConfig.closedLoop.i(kI);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkD(double kD) {
		flywheelMotorConfig.closedLoop.d(kD);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setIndexerkD(double kD) {
		indexerMotorConfig.closedLoop.d(kD);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkS(double kS) {
		flywheelMotorConfig.closedLoop.feedForward.kS(kS);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setIndexerkS(double kS) {
		indexerMotorConfig.closedLoop.feedForward.kS(kS);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkV(double kV) {
		flywheelMotorConfig.closedLoop.feedForward.kV(kV);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setIndexerkV(double kV) {
		indexerMotorConfig.closedLoop.feedForward.kV(kV);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelkA(double kA) {
		flywheelMotorConfig.closedLoop.feedForward.kA(kA);
		flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setIndexerkA(double kA) {
		indexerMotorConfig.closedLoop.feedForward.kA(kA);
		indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setFlywheelVoltage(double volts) {
		flywheelMotor.setVoltage(volts);
	}

	@Override
	public void setIndexerVoltage(double volts) {
		indexerMotor.setVoltage(volts);
	}

	@Override
	public void setFlywheelRpm(double rpm) {
		flywheelController.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
	}

	@Override
	public void setIndexerRpm(double rpm) {
		indexerController.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
	}

	@Override
	public void stopFlywheel() {
		flywheelMotor.stopMotor();
	}

	public void setIndexerSpeed(double speed) {
		indexerMotor.set(speed);
	}

	@Override
	public void stopIndexer() {
		indexerMotor.stopMotor();
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.flywheelPosition.mut_replace(flywheelEncoder.getPosition(), Rotations);
		inputs.indexerPosition.mut_replace(indexerEncoder.getPosition(), Rotations);

		inputs.flywheelVelocity.mut_replace(indexerEncoder.getVelocity(), RPM);
		inputs.indexerVelocity.mut_replace(indexerEncoder.getVelocity(), RPM);

		inputs.flywheelAppliedVolts.mut_replace(flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage(), Volts);
		inputs.indexerAppliedVolts.mut_replace(indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage(), Volts);

		inputs.flywheelCurrent.mut_replace(flywheelMotor.getOutputCurrent(), Amps);
		inputs.indexerCurrent.mut_replace(indexerMotor.getOutputCurrent(), Amps);

		inputs.flywheelTemp.mut_replace(flywheelMotor.getMotorTemperature(), Celsius);
		inputs.indexerTemp.mut_replace(indexerMotor.getMotorTemperature(), Celsius);
	}
}
