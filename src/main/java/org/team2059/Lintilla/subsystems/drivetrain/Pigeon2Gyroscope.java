package org.team2059.Lintilla.subsystems.drivetrain;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyroscope implements GyroscopeIO {
	private final Pigeon2 pigeon;

	public Pigeon2Gyroscope(int canId) {
		pigeon = new Pigeon2(canId);
	}

	public Pigeon2Gyroscope(int canId, CANBus canbus) {
		pigeon = new Pigeon2(canId, canbus);
	}

	@Override
	public void reset() {
		pigeon.reset();
	}

	@Override
	public Rotation2d getRotation2d() {
		return pigeon.getRotation2d();
	}

	@Override
	public void updateInputs(GyroscopeIOInputs inputs) {
		inputs.yaw.mut_replace(pigeon.getYaw().getValue());
		inputs.pitch.mut_replace(pigeon.getPitch().getValue());
		inputs.roll.mut_replace(pigeon.getRoll().getValue());
	}
}