package org.team2059.Lintilla.commands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import org.team2059.Lintilla.subsystems.vision.LocalizationSystem;

import java.util.function.Supplier;

/**
 * Command that calculates the QuestNav offset relative to robot center.
 * <p>
 * The goal is for the driver to rotate in a perfect circle while the operator
 * holds a button that runs this command, and a Transform2d offset is output.
 */
public class QnavCalibrationCommand extends Command {

	private final Supplier<Pose3d> getQnavRawPose;
	private final Supplier<Pose2d> getEstimatedPose;

	// Set this value in radians as to how much data we will accept.
	// Generally, you want somewhere between 1 and 1.5 full rotations.
	private final double targetRotationRads = 2 * Math.PI;

	// Regression accumulators
	private double sum_x = 0, sum_y = 0;
	private double sum_x2, sum_y2 = 0, sum_xy = 0;
	private double sum_xz = 0, sum_yz = 0, sum_z = 0;
	private int n = 0;

	// Rotation tracking
	private Rotation2d lastRotation;
	private double accumulatedRotationRads = 0;

	public QnavCalibrationCommand(
	  Supplier<Pose3d> getQnavRawPose,
	  Supplier<Pose2d> getEstimatedPose
	) {
		this.getQnavRawPose = getQnavRawPose;
		this.getEstimatedPose = getEstimatedPose;
	}

	@Override
	public void initialize() {

		// Set the currently-reported Quest raw pose to zero
		// This helps prevent the old offset from being added to the new offset
		LocalizationSystem.getInstance().setQnavRawPose(Pose3d.kZero);

		// Reset accumulators
		sum_x = 0;
		sum_y = 0;
		sum_x2 = 0;
		sum_y2 = 0;
		sum_xy = 0;
		sum_xz = 0;
		sum_yz = 0;
		sum_z = 0;
		n = 0;
		lastRotation = getEstimatedPose.get().getRotation();
		accumulatedRotationRads = 0;

		System.out.println("Qnav offset calibration begin");
	}

	@Override
	public void execute() {
		// Track how far we've spun, and add that to our accumulator
		Rotation2d currentRotation = getEstimatedPose.get().getRotation();
		double deltaRads = currentRotation.minus(lastRotation).getRadians();
		accumulatedRotationRads += Math.abs(deltaRads);
		lastRotation = currentRotation;

		// Sample Quest position for this loop run
		Pose3d qnavPose = getQnavRawPose.get();
		double x = qnavPose.getX();
		double y = qnavPose.getY();
		double z_val = (x * x) + (y * y); // represents x^2 + y^2 for the regression

		// Update the least-squares matrix accumulators
		sum_x += x;
		sum_y += y;
		sum_x2 += (x * x);
		sum_y2 += (y * y);
		sum_xy += (x * y);
		sum_xz += (x * z_val);
		sum_yz += (y * z_val);
		sum_z += z_val;
		n++;
	}

	@Override
	public boolean isFinished() {
		// Finish once we've spun the target amount
		return accumulatedRotationRads >= targetRotationRads;
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted || n < 10) {
			System.out.println("Calibration interrupted or not enough data gathered");
			return;
		}

		// Construct matrices for U^T U * beta = U^T Z
		Matrix<N3, N3> UTU = new Matrix<>(Nat.N3(), Nat.N3());
		UTU.set(0, 0, sum_x2);
		UTU.set(0, 1, sum_xy);
		UTU.set(0, 2, sum_x);
		UTU.set(1, 0, sum_xy);
		UTU.set(1, 1, sum_y2);
		UTU.set(1, 2, sum_y);
		UTU.set(2, 0, sum_x);
		UTU.set(2, 1, sum_y);
		UTU.set(2, 2, n);

		Matrix<N3, N1> UTZ = new Matrix<>(Nat.N3(), Nat.N1());
		UTZ.set(0, 0, sum_xz);
		UTZ.set(1, 0, sum_yz);
		UTZ.set(2, 0, sum_z);

		try {
			// Solve for beta [A, B, C]^T
			Matrix<N3, N1> beta = UTU.solve(UTZ);

			double A = beta.get(0, 0);
			double B = beta.get(1, 0);

			// Center of the traced circle in Quest's local frame
			double centerX = A / 2.0;
			double centerY = B / 2.0;

			// If the Quest traces a circle around the robot center, the robot center
			// is at (centerX, centerY) in the Quest's startup coordinate space.
			// Therefore, the Quest's position relative to the robot center is the inverse.
			Translation2d questOffset = new Translation2d(centerY, -centerX);

			System.out.println("=== QuestNav Calibration Complete ===");
			System.out.printf("Samples taken: %d%n", n);
			System.out.printf("Robot Center in Quest Frame: (%f, %f)%n", centerX, centerY);
			System.out.printf("Calculated Quest Offset: X=%f meters, Y=%f meters%n",
			  questOffset.getX(), questOffset.getY());

		} catch (Exception e) {
			System.out.println("Matrix solve failed: " + e.getMessage());
		}
	}
}
