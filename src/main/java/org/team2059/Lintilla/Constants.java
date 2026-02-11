// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class OperatorConstants {

		// Sets whether tunable numbers can be changed. If false, only defaults will be used.
		public static final boolean tuningMode = true;

		/* ===== */
		/* PORTS */
		/* ===== */

		public static final int logitechPort = 0;
		public static final int buttonBoxPort = 1;

		/* ==== */
		/* AXES */
		/* ==== */

		public static final int JoystickTranslationAxis = 1;
		public static final int JoystickStrafeAxis = 0;
		public static final int JoystickRotationAxis = 2;
		public static final int JoystickSliderAxis = 3;

		/* ======= */
		/* BUTTONS */
		/* ======= */

		// We'll add these later once the button box is finalized
		public static final int JoystickResetHeading = 5;
		public static final int JoystickRobotRelative = 6;
		public static final int JoystickInvertedDrive = 4;
		public static final int JoystickStrafeOnly = 3;
	}

	public static class AutoConstants {

		// Feedback constants for x & y translation in auto.
		public static final double kAutoTranslationP = 5.0;
		public static final double kAutoTranslationD = 0;

		// Feedback constants for theta (rotation) in auto.
		public static final double kAutoRotationP = 5.0;
		public static final double kAutoRotationD = 0.0;

		/* FOR ROBOTCONFIG AUTO STUFF... */
		/* Not used right now. */
		// public static final double kMass = 30;
		// public static final double kMomentOfIntertia = 3;

		// // CoF taken from https://www.chiefdelphi.com/t/coefficient-of-friction/467778
		// public static final double kWheelCoF = 1.542; // Coefficient of friction of wheels
		// public static final int driveCurrentLimit = 40;
		// public static final int turnCurrentLimit = 20;
	}

	public static final class DrivetrainConstants {
		// Global maximums
		public static final double maxVelocity = 5; // meters/sec
		public static final double maxAcceleration = 10; // meters/sec^2
		public static final double maxAngularVelocity = 2 * Math.PI; // rad/sec
		public static final double maxAngularAcceleration = 4 * Math.PI; // rad/sec^2
		// Teleop max speeds
		public static final double kTeleDriveMaxSpeed = 4;
		public static final double kTeleDriveMaxAngularSpeed = Math.PI;

		/*
		 * MK5n Gear Ratios
		 * Drive:
		 * - R1: 7.03:1
		 * - R2: 6.03:1
		 * - R3: 5.27:1
		 *
		 * Rotation:
		 * - 287:11
		 *
		 * Wheel Diameter: 4 inches, 0.1016 meters
		 */

		public static final Distance wheelBase = Inches.of(25.7); // Distance from center of wheels on side
		public static final Distance trackWidth = Inches.of(19); // Distance between front wheels (like train track)

		// Kinematics give each module relative to center. X is forward/backward and Y is left/right.
		public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
		  new Translation2d(wheelBase.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0), // front right (+,+)
		  new Translation2d(wheelBase.in(Meters) / 2.0, -trackWidth.in(Meters) / 2.0), // back right (+,-)
		  new Translation2d(-wheelBase.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0), // front left (-,+)
		  new Translation2d(-wheelBase.in(Meters) / 2.0, -trackWidth.in(Meters) / 2.0) // back left (-,-)
		);

		public static final double drivePositionConversionFactor = 0.05293297075;
		public static final double driveVelocityConversionFactor = 0.0008822161791;

		public static final double rotationGearRatio = 26.09090909091;

		// TODO: Torque-current control with x44's (azimuth)
		public static final double azimuthkP = 0.0; // An error of 1 rotation results in this much output in amps
		public static final double azimuthkI = 0.0;
		public static final double azimuthkD = 0.0; // A velocity of 1 rotation/sec results in this much output

		// Peak output of any x44 on the swerve modules
		public static final double peakTorqueCurrentAmps = 60;

		/*
		 * CANcoder offsets, in rotations
		 * Instructions for finding these offsets:
		 *
		 * Open Phoenix Tuner X and locate your CANcoder. Run self-test.
		 * Read value from "Absolute Position - No Offset". Add a negative. Paste the value here.
		 */
		public static final double frontRightEncoderOffset = -0.347656;
		public static final double frontLeftEncoderOffset = -0.983887;
		public static final double backRightEncoderOffset = -0.605713;
		public static final double backLeftEncoderOffset = -0.650391;

		// Drive motor inversions
		public static final boolean frontRightInverted = true;
		public static final boolean frontLeftInverted = false;
		public static final boolean backRightInverted = true;
		public static final boolean backLeftInverted = false;
		public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.17821, 1.9047, 0.14686);
		public static double kPRotation = 0.25;
	}

	public static final class VisionConstants {
		public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
		public static final Transform2d ROBOT_TO_QUEST_2D = new Transform2d(0.06445393, 0.23854354, Rotation2d.kCCW_90deg);
		public static final Transform3d ROBOT_TO_QUEST = new Transform3d(ROBOT_TO_QUEST_2D);
		public static Matrix<N3, N1> questNavStdDevs = VecBuilder.fill(
		  0.02, // Trust down to 2 cm in X direction
		  0.02, // Trust down to 2 cm in Y direction
		  0.035 // Trust down to 2 degrees rotational
		);
	}

	public static final class CANConstants {

		public static final CANBus canivore = new CANBus("Deep Control");

		public static final int pigeon2 = 60;

		public static final int frontRightDriveMotor = 1;
		public static final int frontRightTurnMotor = 2;
		public static final int frontRightCancoder = 10;
		public static final int frontLeftDriveMotor = 3;
		public static final int frontLeftTurnMotor = 4;
		public static final int frontLeftCancoder = 20;
		public static final int backRightDriveMotor = 5;
		public static final int backRightTurnMotor = 6;
		public static final int backRightCancoder = 30;
		public static final int backLeftDriveMotor = 7;
		public static final int backLeftTurnMotor = 8;
		public static final int backLeftCancoder = 40;

		public static final int powerDistributionHub = 50;

		public static final int shooterIndexerMotor = 54; // Same ID for both shooters for now (will be changed later)
		public static final int leftShooterFlywheel = 53;
		public static final int rightShooterFlywheel = 55;
		public static final int leftShooterFlywheel = 53;
		public static final int rightShooterFlywheel = -1;

		public static final int collectorTiltMotor = 57;
		public static final int collectorIntakeMotor = 58;
		public static final int conveyorMotor = 59;
	}

	public static final class ShooterConstants {

		public static final boolean leftFlywheelInverted = true;
		public static final boolean rightFlywheelInverted = true;

		public static final double gravitationalAccelerationMpss = 9.80665;
		public static final double hubHeightMeters = 1.83; // End height of trajectory
		public static final double shooterHeightMeters = 0.5; // Start height of trajectory
		public static final double fuelExitAngleRadians = 40 * Math.PI / 180; // RADIANS At what angle does fuel leave the shooter

		/*
		 * Units of Flywheel Constants (Thanks Rev for good docs this year)
		 * - kP: Duty cycle per rotation of error
		 * - kI: Duty cycle per (rotation * ms)
		 * - kD: (Duty cycle * ms) per duty cycle
		 * - kS: Volts
		 * - kV: Volts per RPM
		 * - kA: Volts per RPM/sec
		 */

		public static final double indexerkP = 0.0;
		public static final double indexerkI = 0.0;
		public static final double indexerkD = 0.0;
		public static final double indexerkS = 0;
		public static final double indexerkV = 0;
		public static final double indexerkA = 0;

		public static final double leftkP = 0.0;
		public static final double leftkI = 0.0;
		public static final double leftkD = 0.0;
		public static final double leftkS = 0.071064;
		public static final double leftkV = 0.10564 / 60;
		public static final double leftkA = 0.017298 / 60;

		public static final double rightkP = 0.0;
		public static final double rightkI = 0.0;
		public static final double rightkD = 0.0;
		public static final double rightkS = 0.13506;
		public static final double rightkV = 0.10641 / 60;
		public static final double rightkA = 0.017354 / 60;
	}
}