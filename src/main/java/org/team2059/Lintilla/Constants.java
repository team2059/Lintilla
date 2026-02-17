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
import edu.wpi.first.math.geometry.*;
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
		public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

		public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6116, 4.0213);
		public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9014, 4.0213);

		public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
		  -0.30449689,
		  0.16938114,
		  0,
		  new Rotation3d(
			0,
		    0,
		    Math.PI / 2
		  )
		);



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

		public static final int leftShooterIndexerMotor = 54; // Same ID for both shooters for now (will be changed later)
		public static final int rightShooterIndexerMotor = 55;
		public static final int leftShooterFlywheel = 53;
		public static final int rightShooterFlywheel = 56;

		public static final int collectorTiltMotor = 57;
		public static final int collectorIntakeMotor = 58;
		public static final int conveyorMotor = 59;
	}

	public static final class ShooterConstants {

		// Error to tolerate when spinning up to shoot (in RPMs)
		public static final double spinupToleranceRpm = 100;

		/*
		 * Units of Flywheel Constants (Thanks Rev for good docs this year)
		 * - kP: Duty cycle per rotation of error
		 * - kI: Duty cycle per (rotation * ms)
		 * - kD: (Duty cycle * ms) per duty cycle
		 * - kS: Volts
		 * - kV: Volts per RPM
		 * - kA: Volts per RPM/sec
		 */

		public static final double gravitationalAccelerationMpss = 9.80665;
		public static final double hubHeightMeters = 1.83; // End height of trajectory
		public static final double shooterHeightMeters = 0.5; // Start height of trajectory
		public static final double fuelExitAngleRadians = 40 * Math.PI / 180; // RADIANS At what angle does fuel leave the shooter

		// DO NOT TOUCH the following
		public static final double tangentShooterAngle = Math.tan(fuelExitAngleRadians);
		public static final double cosineShooterAngleSquared = Math.pow(Math.cos(fuelExitAngleRadians), 2);
		public static final double dY = hubHeightMeters - shooterHeightMeters;
		public static final double minimumShotDistanceMeters = dY / tangentShooterAngle;

		// LEFT SHOOTER CONSTANTS
		public static final boolean leftFlywheelInverted = true;
		public static final boolean leftIndexerInverted = true;

		public static final double leftIndexerkP = 0.0;
		public static final double leftIndexerkI = 0.0;
		public static final double leftIndexerkD = 0.0;
		public static final double leftIndexerkS = 0.079892;
		public static final double leftIndexerkV = 0.10559/60;
		public static final double leftIndexerkA = 0.0073533/60;

		public static final double leftFlywheelkP = 0.0;
		public static final double leftFlywheelkI = 0.0;
		public static final double leftFlywheelkD = 0.0;
		public static final double leftFlywheelkS = 0.089332;
		public static final double leftFlywheelkV = 0.10669/60;
		public static final double leftFlywheelkA = 0.020699/60;

		// RIGHT SHOOTER CONSTANTS
		public static final boolean rightFlywheelInverted = false;
		public static boolean rightIndexerInverted = false;

		public static final double rightIndexerkP = 0.0;
		public static final double rightIndexerkI = 0.0;
		public static final double rightIndexerkD = 0.0;
		public static final double rightIndexerkS = 0.080536;
		public static final double rightIndexerkV = 0.10572/60;
		public static final double rightIndexerkA = 0.008408/60;

		public static final double rightFlywheelkP = 0.0;
		public static final double rightFlywheelkI = 0.0;
		public static final double rightFlywheelkD = 0.0;
		public static final double rightFlywheelkS = 0.04773;
		public static final double rightFlywheelkV = 0.10424/60;
		public static final double rightFlywheelkA = 0.020809/60;
	}

	public static final class CollectorConstants {
		public static final double kPTilt = 0.6;
		public static final double kITilt = 0;
		public static final double kDTilt = 0;
		public static final double kCosTilt = 0.35487;
		public static final double kSTilt = 0.49117;
		public static final double kVTilt = 1.234/60;
		public static final double kATilt = 0;

		public static final double thruBoreOffset = 0.91;

		public static final double thruBoreOut = 0;
		public static final double thruBoreIn = 0.25;
	}
}