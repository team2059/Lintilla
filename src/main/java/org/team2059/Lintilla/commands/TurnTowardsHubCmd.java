package org.team2059.Lintilla.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.Constants;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.util.LoggedTunableNumber;

public class TurnTowardsHubCmd extends Command {
	private final Drivetrain drivetrain;
	private final Translation2d hub;
	private final PIDController controller;

	private final LoggedTunableNumber kP = new LoggedTunableNumber("HubTurnKp", 0);
	private final LoggedTunableNumber kI = new LoggedTunableNumber("HubTurnKi", 0);
	private final LoggedTunableNumber kD = new LoggedTunableNumber("HubTurnKd", 0);

	public TurnTowardsHubCmd(Drivetrain drivetrain, boolean blueHub) {
		this.drivetrain = drivetrain;

		// Set hub based on requested color
		hub = blueHub ? Constants.VisionConstants.BLUE_HUB_CENTER : Constants.VisionConstants.RED_HUB_CENTER;

		// Initialize PID controller using tunable numbers created earlier
		controller = new PIDController(kP.get(), kI.get(), kD.get());

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		controller.setSetpoint(0);
		controller.setTolerance(0.087);
	}

	@Override
	public void execute() {
		// Check for updated TunableNumbers and take action
		LoggedTunableNumber.ifChanged(
		  hashCode(),
		  () -> {
			  controller.setPID(kP.get(), kI.get(), kD.get());
		  },
		  kP, kI, kD
		);

		Translation2d diff = hub.minus(drivetrain.getEstimatedPose().getTranslation());
		double currentAngle = Math.atan2(diff.getY(), diff.getX());

		double nextOutput = controller.calculate(currentAngle);

		drivetrain.drive(0, 0, nextOutput, false);
	}

	@Override
	public boolean isFinished() {
		return controller.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
	}
}