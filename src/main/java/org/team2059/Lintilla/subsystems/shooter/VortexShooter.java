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
import static org.team2059.Lintilla.Constants.CANConstants.*;

public class VortexShooter implements ShooterIO {

    private final SparkFlex leftDrumMotor;
    private final SparkFlex rightDrumMotor;

    private final SparkFlex leftIndexerMotor;
    private final SparkFlex rightIndexerMotor;

    private final SparkFlexConfig leftDrumMotorConfig =
        new SparkFlexConfig();

    private final SparkFlexConfig rightDrumMotorConfig =
        new SparkFlexConfig();

    private final SparkFlexConfig leftIndexerMotorConfig =
        new SparkFlexConfig();

    private final SparkFlexConfig rightIndexerMotorConfig =
        new SparkFlexConfig();

    private final SparkClosedLoopController drumController;
    private final SparkClosedLoopController indexerController;

    private final RelativeEncoder drumEncoder;
    private final RelativeEncoder indexerEncoder;

    public VortexShooter(
        int leftDrumMotorCanId, int leftIndexerMotorCanId, int rightDrumMotorCanId, int rightIndexerMotorCanId, boolean leftDrumMotorInverted, boolean leftIndexerMotorInverted, 
		double kPDrum, double kIDrum, double kDDrum, double kSDrum, double kVDrum, double kADrum,
        double kPIndexer, double kIIndexer, double kDIndexer, double kSIndexer, double kVIndexer, double kAIndexer
    ) {
        leftDrumMotor = new SparkFlex(
            leftDrumMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        );

        rightDrumMotor = new SparkFlex(
            rightDrumMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        );

        leftIndexerMotor = new SparkFlex(
            leftIndexerMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        );

        rightIndexerMotor = new SparkFlex(
            rightIndexerMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        );

        leftDrumMotorConfig
            .inverted(leftDrumMotorInverted)
            .idleMode(SparkBaseConfig.IdleMode.kCoast);

        rightDrumMotorConfig
            .inverted(!leftDrumMotorInverted)
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
			.follow(leftDrumMotorCanId);

        leftIndexerMotorConfig
            .inverted(leftIndexerMotorInverted)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);

        rightIndexerMotorConfig
            .inverted(!leftIndexerMotorInverted)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
			.follow(leftIndexerMotorCanId);

        leftDrumMotorConfig.closedLoop
            .pid(kPDrum, kIDrum, kDDrum)
            .feedForward
            .kS(kSDrum)
            .kV(kVDrum)
            .kA(kADrum);

        leftIndexerMotorConfig.closedLoop
            .pid(kPIndexer, kIIndexer, kDIndexer)
            .feedForward
            .kS(kSIndexer)
            .kV(kVIndexer)
            .kA(kAIndexer);

        leftDrumMotorConfig.signals
            .primaryEncoderPositionPeriodMs(REV_POSITION_PERIOD_MS)
            .primaryEncoderVelocityPeriodMs(REV_VELOCITY_PERIOD_MS)
            .appliedOutputPeriodMs(REV_APPLIED_OUTPUT_PERIOD_MS)
            .outputCurrentPeriodMs(REV_OUTPUT_CURRENT_PERIOD_MS)
            .motorTemperaturePeriodMs(REV_MOTOR_TEMP_PERIOD_MS)
            .faultsPeriodMs(REV_MOTOR_FAULTS_PERIOD_MS);

        rightDrumMotorConfig.signals
            .primaryEncoderPositionPeriodMs(REV_POSITION_PERIOD_MS)
            .primaryEncoderVelocityPeriodMs(REV_VELOCITY_PERIOD_MS)
            .appliedOutputPeriodMs(REV_APPLIED_OUTPUT_PERIOD_MS)
            .outputCurrentPeriodMs(REV_OUTPUT_CURRENT_PERIOD_MS)
            .motorTemperaturePeriodMs(REV_MOTOR_TEMP_PERIOD_MS)
            .faultsPeriodMs(REV_MOTOR_FAULTS_PERIOD_MS);

        leftIndexerMotorConfig.signals
            .primaryEncoderPositionPeriodMs(REV_POSITION_PERIOD_MS)
            .primaryEncoderVelocityPeriodMs(REV_VELOCITY_PERIOD_MS)
            .appliedOutputPeriodMs(REV_APPLIED_OUTPUT_PERIOD_MS)
            .outputCurrentPeriodMs(REV_OUTPUT_CURRENT_PERIOD_MS)
            .motorTemperaturePeriodMs(REV_MOTOR_TEMP_PERIOD_MS)
            .faultsPeriodMs(REV_MOTOR_FAULTS_PERIOD_MS);

        rightIndexerMotorConfig.signals
            .primaryEncoderPositionPeriodMs(REV_POSITION_PERIOD_MS)
            .primaryEncoderVelocityPeriodMs(REV_VELOCITY_PERIOD_MS)
            .appliedOutputPeriodMs(REV_APPLIED_OUTPUT_PERIOD_MS)
            .outputCurrentPeriodMs(REV_OUTPUT_CURRENT_PERIOD_MS)
            .motorTemperaturePeriodMs(REV_MOTOR_TEMP_PERIOD_MS)
            .faultsPeriodMs(REV_MOTOR_FAULTS_PERIOD_MS);

        leftDrumMotor.configure(
            leftDrumMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        rightDrumMotor.configure(
            rightDrumMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        leftIndexerMotor.configure(
            leftIndexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        rightIndexerMotor.configure(
            rightIndexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        leftDrumMotor.clearFaults();
        rightDrumMotor.clearFaults();
        leftIndexerMotor.clearFaults();
        rightIndexerMotor.clearFaults();

        drumController = leftDrumMotor.getClosedLoopController();
        indexerController = leftIndexerMotor.getClosedLoopController();

        drumEncoder = leftDrumMotor.getEncoder();
        indexerEncoder = leftIndexerMotor.getEncoder();
    }

    @Override
    public void setDrumkP(double kP) {
        leftDrumMotorConfig.closedLoop.p(kP);
        leftDrumMotor.configure(
            leftDrumMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setDrumkI(double kI) {
        leftDrumMotorConfig.closedLoop.i(kI);
        leftDrumMotor.configure(
            leftDrumMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setDrumkD(double kD) {
        leftDrumMotorConfig.closedLoop.d(kD);
        leftDrumMotor.configure(
            leftDrumMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setDrumkS(double kS) {
        leftDrumMotorConfig.closedLoop.feedForward.kS(kS);
        leftDrumMotor.configure(
            leftDrumMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setDrumkV(double kV) {
        leftDrumMotorConfig.closedLoop.feedForward.kV(kV);
        leftDrumMotor.configure(
            leftDrumMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setDrumkA(double kA) {
        leftDrumMotorConfig.closedLoop.feedForward.kA(kA);
        leftDrumMotor.configure(
            leftDrumMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

	@Override
	public void setRightMotorSpeed(double speed) {
		rightDrumMotor.set(speed);
	}

    @Override
    public void setIndexerkP(double kP) {
        leftIndexerMotorConfig.closedLoop.p(kP);
        leftIndexerMotor.configure(
            leftIndexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setIndexerkI(double kI) {
        leftIndexerMotorConfig.closedLoop.i(kI);
        leftIndexerMotor.configure(
            leftIndexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setIndexerkD(double kD) {
        leftIndexerMotorConfig.closedLoop.d(kD);
        leftIndexerMotor.configure(
            leftIndexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setIndexerkS(double kS) {
        leftIndexerMotorConfig.closedLoop.feedForward.kS(kS);
        leftIndexerMotor.configure(
            leftIndexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setIndexerkV(double kV) {
        leftIndexerMotorConfig.closedLoop.feedForward.kV(kV);
        leftIndexerMotor.configure(
            leftIndexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setIndexerkA(double kA) {
        leftIndexerMotorConfig.closedLoop.feedForward.kA(kA);
        leftIndexerMotor.configure(
            leftIndexerMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    @Override
    public void setDrumVoltage(double volts) {
        leftDrumMotor.setVoltage(volts);
    }

    @Override
    public void setIndexerVoltage(double volts) {
        leftIndexerMotor.setVoltage(volts);
    }

    @Override
    public void setDrumRpm(double rpm) {
        drumController.setSetpoint(
            rpm,
            SparkBase.ControlType.kVelocity
        );
    }

    @Override
    public void setIndexerRpm(double rpm) {
        indexerController.setSetpoint(
            rpm,
            SparkBase.ControlType.kVelocity
        );
    }

    @Override
    public void setDrumSpeed(double speed) {
        leftDrumMotor.set(speed);
    }

    @Override
    public void setIndexerSpeed(double speed) {
        leftIndexerMotor.set(speed);
    }

    @Override
    public void stopDrum() {
        leftDrumMotor.stopMotor();
    }

    @Override
    public void stopIndexer() {
        leftIndexerMotor.stopMotor();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.drumPosition.mut_replace(drumEncoder.getPosition(), Rotations);

        inputs.drumVelocity.mut_replace(drumEncoder.getVelocity(), RPM);

        inputs.drumAppliedVolts.mut_replace(
            leftDrumMotor.getAppliedOutput()
                * leftDrumMotor.getBusVoltage(),
            Volts
        );

        inputs.drumCurrent.mut_replace(
            leftDrumMotor.getOutputCurrent(),
            Amps
        );

        inputs.drumTemp.mut_replace(
            leftDrumMotor.getMotorTemperature(),
            Celsius
        );

        inputs.drumFollowerCurrent.mut_replace(
            rightDrumMotor.getOutputCurrent(),
            Amps
        );

        inputs.drumFollowerTemp.mut_replace(
            rightDrumMotor.getMotorTemperature(),
            Celsius
        );

        inputs.indexerPosition.mut_replace(
            indexerEncoder.getPosition(),
            Rotations
        );

        inputs.indexerVelocity.mut_replace(
            indexerEncoder.getVelocity(),
            RPM
        );

        inputs.indexerAppliedVolts.mut_replace(
            leftIndexerMotor.getAppliedOutput()
                * leftIndexerMotor.getBusVoltage(),
            Volts
        );

        inputs.indexerCurrent.mut_replace(
            leftIndexerMotor.getOutputCurrent(),
            Amps
        );

        inputs.indexerTemp.mut_replace(
            leftIndexerMotor.getMotorTemperature(),
            Celsius
        );

        inputs.indexerFollowerCurrent.mut_replace(
            rightIndexerMotor.getOutputCurrent(),
            Amps
        );

        inputs.indexerFollowerTemp.mut_replace(
            rightIndexerMotor.getMotorTemperature(),
            Celsius
        );
    }
}
