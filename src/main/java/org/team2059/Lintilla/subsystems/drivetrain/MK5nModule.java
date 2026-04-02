package org.team2059.Lintilla.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.team2059.Lintilla.Constants;
import org.team2059.Lintilla.Constants.CANConstants;
import org.team2059.Lintilla.util.SwerveUtilities;

import static edu.wpi.first.units.Units.*;
import static org.team2059.Lintilla.Constants.CANConstants.*;
import static org.team2059.Lintilla.Constants.DrivetrainConstants.*;

public class MK5nModule implements SwerveModuleIO {
	private final SparkFlex driveMotor;
	private final SparkFlexConfig driveMotorConfig = new SparkFlexConfig();
	private final RelativeEncoder driveEncoder;

	private final TalonFX azimuthMotor;
	private final TalonFXConfiguration azimuthMotorConfig = new TalonFXConfiguration();
	private final CANcoder canCoder;
	private final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
	private final PositionVoltage azimuthPositionReq = new PositionVoltage(0).withSlot(0);

	private final SwerveModuleState currentState = new SwerveModuleState();

	public MK5nModule(
	  int driveMotorCanId,
	  int azimuthMotorCanId,
	  int canCoderCanId,
	  double cancoderOffset,
	  boolean driveInverted
	) {

		// Configure drive motor
		driveMotor = new SparkFlex(driveMotorCanId, SparkLowLevel.MotorType.kBrushless);

		driveMotorConfig
		  .inverted(driveInverted)
		  .idleMode(SparkBaseConfig.IdleMode.kBrake);

		driveMotorConfig.encoder
		  .positionConversionFactor(Constants.DrivetrainConstants.DRIVE_POSITION_FACTOR)
		  .velocityConversionFactor(Constants.DrivetrainConstants.DRIVE_VELOCITY_FACTOR);

		driveMotorConfig.signals
		  .primaryEncoderPositionPeriodMs(REV_POSITION_PERIOD_MS)
		  .primaryEncoderVelocityPeriodMs(REV_VELOCITY_PERIOD_MS)
		  .appliedOutputPeriodMs(REV_APPLIED_OUTPUT_PERIOD_MS)
		  .outputCurrentPeriodMs(REV_OUTPUT_CURRENT_PERIOD_MS)
		  .motorTemperaturePeriodMs(REV_MOTOR_TEMP_PERIOD_MS)
		  .faultsPeriodMs(REV_MOTOR_FAULTS_PERIOD_MS);

		driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		driveEncoder = driveMotor.getEncoder();

		driveMotor.clearFaults();

		// Configure cancoder
		canCoder = new CANcoder(canCoderCanId, CANConstants.CANIVORE);

		canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
		canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		canCoderConfig.MagnetSensor.withMagnetOffset(cancoderOffset);

		canCoder.getConfigurator().apply(canCoderConfig);

		// Configure turn motor
		azimuthMotor = new TalonFX(azimuthMotorCanId, CANConstants.CANIVORE);

		azimuthMotorConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
		azimuthMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		azimuthMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
		azimuthMotorConfig.Feedback.RotorToSensorRatio = Constants.DrivetrainConstants.ROTATION_GEAR_RATIO;

		azimuthMotorConfig.Slot0.kP = ROTATION_P;
		azimuthMotorConfig.Slot0.kI = ROTATION_I;
		azimuthMotorConfig.Slot0.kD = ROTATION_D;

		azimuthMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

		azimuthMotor.getConfigurator().apply(azimuthMotorConfig);

		azimuthMotor.clearStickyFaults();
	}

	@Override
	public void resetEncoders() {
		// No longer resets rotation encoder (there's no need, it's absolute)
		driveEncoder.setPosition(0);
	}

	@Override
	public void stop() {
		driveMotor.stopMotor();
		azimuthMotor.stopMotor();
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveMotor.setVoltage(volts);
	}

	@Override
	public void setAzimuthVoltage(double volts) {
		azimuthMotor.setVoltage(volts);
	}

	@Override
	public SwerveModuleState getState() {
		currentState.speedMetersPerSecond = driveEncoder.getVelocity();
		currentState.angle = Rotation2d.fromRadians(canCoder.getAbsolutePosition().getValue().in(Radians));

		return currentState;
	}

	@Override
	public void setState(SwerveModuleState targetState) {
		// Deadband
		if (Math.abs(targetState.speedMetersPerSecond) < 0.001) {
			stop();
			return;
		}

		Rotation2d cancoderAbsPos = Rotation2d.fromRadians(canCoder.getAbsolutePosition().getValue().in(Radians));

		// Optimize angle
		targetState = SwerveUtilities.optimize(targetState, cancoderAbsPos);

		// PID-controlled rotation - temporary
		azimuthMotor.setControl(azimuthPositionReq.withPosition(targetState.angle.getRotations()));

		driveMotor.setVoltage(DRIVE_FEEDFORWARD.calculate(targetState.speedMetersPerSecond));
	}

	private double getDriveAppliedVolts() {
		return (driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		inputs.drivePos.mut_replace(driveEncoder.getPosition(), Meters);
		inputs.driveVel.mut_replace(driveEncoder.getVelocity(), MetersPerSecond);
		inputs.driveAppliedVolts.mut_replace(getDriveAppliedVolts(), Volts);
		inputs.driveCurrent.mut_replace(driveMotor.getOutputCurrent(), Amps);
		inputs.driveTemp.mut_replace(driveMotor.getMotorTemperature(), Celsius);

		inputs.azimuthAbsolutePos.mut_replace(canCoder.getAbsolutePosition().getValue());
		inputs.azimuthVel.mut_replace(canCoder.getVelocity().getValue());
		inputs.azimuthAppliedVolts.mut_replace(azimuthMotor.getMotorVoltage().getValue());
		inputs.azimuthCurrent.mut_replace(azimuthMotor.getTorqueCurrent().getValue());
		inputs.azimuthTemp.mut_replace(azimuthMotor.getDeviceTemp().getValue());
	}
}