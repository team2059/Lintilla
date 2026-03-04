// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public final class Constants {
	/**
	 * Constants class for operator buttons and joystick axes
	 */
	public static final class OperatorConstants {

		// Sets whether tunable numbers can be changed. If false, only defaults will be used.
		public static final boolean tuningMode = true;

		/* ===== */
		/* PORTS */
		/* ===== */

		public static final int LOGITECH_PORT = 0;
		public static final int BUTTON_BOX_PORT = 1;

		/* ==== */
		/* AXES */
		/* ==== */

		public static final int TRANSLATION_AXIS = 1;
		public static final int STRAFE_AXIS = 0;
		public static final int ROTATION_AXIS = 2;
		public static final int SLIDER_AXIS = 3;

		/* ======= */
		/* BUTTONS */
		/* ======= */

		public static final int RESET_HEADING = 5;
		public static final int ROBOT_RELATIVE = 6;
		public static final int INVERT_DRIVE = 4;
		public static final int STRAFE_ONLY = 3;
		public static final int HUB_ALIGN = 2;
		public static final int SPINUP_SHOOT_DISTANCE = 1;
		public static final int SPINUP_SHOOT_FIXED = 2;
		public static final int COLLECTOR_OUT_INTAKE = 5;
		public static final int COLLECTOR_IN = 6;
		public static final int COLLECTOR_UNJAM = 7;
		public static final int COLLECTOR_INTAKE = 8;

		/* ======== */
		/* SWITCHES */
		/* ======== */

		public static final int QUEST_MEASUREMENT_SWITCH = 13;
		public static final int PHOTONVISION_MEASUREMENT_SWITCH = 14;
	}

	/**
	 * Class that holds gains and configuration for autonomous configuration (i.e. via PathPlanner).
	 */
	public static class AutoConstants {

		// Feedback constants for x & y translation in auto.
		public static final double AUTO_TRANSLATION_P = 5.0;
		public static final double AUTO_TRANSLATION_D = 0;

		// Feedback constants for theta (rotation) in auto.
		public static final double AUTO_ROTATION_P = 5.0;
		public static final double AUTO_ROTATION_D = 0.0;
	}

	/**
	 * Constants class for the Drivetrain class and respective SwerveModules. Contains all object related to kinematics.
	 */
	public static final class DrivetrainConstants {
		// Global maximums
		public static final double MAX_VELOCITY = 3; // meters/sec
		public static final double MAX_ACCELERATION = 3; // meters/sec^2
		public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI; // rad/sec
		public static final double MAX_ANGULAR_ACCELERATION = 4 * Math.PI; // rad/sec^2
		// Teleop max speeds
		public static final double TELE_DRIVE_MAX_SPEED = 4;
		public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = Math.PI;

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

		public static final Distance WHEEL_BASE = Inches.of(26); // Distance from center of wheels on side
		public static final Distance TRACK_WIDTH = Inches.of(19); // Distance between front wheels (like train track)

		// Kinematics give each module relative to center. X is forward/backward and Y is left/right.
		public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
		  new Translation2d(WHEEL_BASE.in(Meters) / 2.0, TRACK_WIDTH.in(Meters) / 2.0), // front right (+,+)
		  new Translation2d(WHEEL_BASE.in(Meters) / 2.0, -TRACK_WIDTH.in(Meters) / 2.0), // back right (+,-)
		  new Translation2d(-WHEEL_BASE.in(Meters) / 2.0, TRACK_WIDTH.in(Meters) / 2.0), // front left (-,+)
		  new Translation2d(-WHEEL_BASE.in(Meters) / 2.0, -TRACK_WIDTH.in(Meters) / 2.0) // back left (-,-)
		);

		public static final double DRIVE_POSITION_FACTOR = 0.05293297075;
		public static final double DRIVE_VELOCITY_FACTOR = 0.0008822161791;

		public static final double ROTATION_GEAR_RATIO = 26.09090909091;

		/*
		 * CANcoder offsets, in rotations
		 * Instructions for finding these offsets:
		 *
		 * Open Phoenix Tuner X and locate your CANcoder. Run self-test.
		 * Read value from "Absolute Position - No Offset". Add a negative. Paste the value here.
		 */
		public static final double FR_ENCODER_OFFFSET = -0.847900;
		public static final double FL_ENCODER_OFFSET = -0.981934;
		public static final double BR_ENCODER_OFFSET = -0.605225;
		public static final double BL_ENCODER_OFFSET = -0.150635;

		// Drive motor inversions
		public static final boolean FR_INVERTED = false;
		public static final boolean FL_INVERTED = false;
		public static final boolean BR_INVERTED = true;
		public static final boolean BL_INVERTED = true;
		public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.17821, 1.9047, 0.14686);
		public static double ROTATION_P = 0.5;
	}

	/**
	 * Constants for anything vision-related, including standard deviations, pose offsets,
	 * and poses of field elements for tracking
	 */
	public static final class VisionConstants {

		public static final Transform2d SHOOTER_OFFSET = new Transform2d(-Units.inchesToMeters(10), 0, Rotation2d.kZero);

		public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
		  -0.26545065,
		  0.18207046,
		  0,
		  new Rotation3d(
			0,
			0,
			Math.PI / 2
		  )
		);

		public static final Transform3d ROBOT_TO_PV = new Transform3d(
		  -Units.inchesToMeters(5),
		  -Units.inchesToMeters(1.375),
		  Units.inchesToMeters(19),
		  new Rotation3d(
			0,
			Units.degreesToRadians(-56),
			0
		  )
		);

		public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

		public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6116, 4.0213);
		public static final Translation2d BLUE_HUB_BACK = new Translation2d(5.2342, 4.0213);

		public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9014, 4.0213);
		public static final Translation2d RED_HUB_BACK = new Translation2d(11.3044, 4.0213);

		public static final Translation2d BLUE_TOWER_CENTER = new Translation2d(1.1434, 3.7457);
		public static final Translation2d RED_TOWER_CENTER = new Translation2d(15.3952, 4.3236);

		public static final String PV_CAM_NAME = "Bcam9782";
		// The standard deviations of our estimated poses, which affect correction rate
		public static final Matrix<N3, N1> PV_SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 8);
		public static final Matrix<N3, N1> PV_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
		public static Matrix<N3, N1> QNAV_STD_DEVS = VecBuilder.fill(
		  0.02, // Trust down to 2 cm in X direction
		  0.02, // Trust down to 2 cm in Y direction
		  0.035 // Trust down to 2 degrees rotational
		);

		/**
		 * Gets the Translation2d of the current Alliance Hub
		 *
		 * @return alliance Hub coordinates
		 */
		public static Translation2d getHubTranslation() {
			Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

			if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
				return RED_HUB_CENTER;
			} else {
				return BLUE_HUB_CENTER;
			}
		}

		public static final double QUESTNAV_FAILURE_THRESHOLD = 6.0;

		/**
		 * The threshold for the error between the best AprilTag pose estimate and the QuestNav pose measurements for the
		 * QuestNav pose to be considered valid
		 */
		public static final Distance QUESTNAV_APRILTAG_ERROR_THRESHOLD = Meters.of(0.5);
	}

	/**
	 * Constants class for anything that interfaces via CAN. All IDs can be found here.
	 */
	public static final class CANConstants {

		public static final CANBus CANIVORE = new CANBus("Deep Control");

		public static final int PIGEON = 60;

		public static final int FR_DRIVE = 1;
		public static final int FR_TURN = 2;
		public static final int FR_CANCODER = 10;
		public static final int FL_DRIVE = 3;
		public static final int FL_TURN = 4;
		public static final int FL_CANCODER = 20;
		public static final int BR_DRIVE = 5;
		public static final int BR_TURN = 6;
		public static final int BR_CANCODER = 30;
		public static final int BL_DRIVE = 7;
		public static final int BL_TURN = 8;
		public static final int BL_CANCODER = 40;

		public static final int PDH = 50;

		public static final int LEFT_SHOOTER_INDEXER = 54; // Same ID for both shooters for now (will be changed later)
		public static final int RIGHT_SHOOTER_INDEXER = 55;
		public static final int LEFT_SHOOTER_FLYWHEEL = 53;
		public static final int RIGHT_SHOOTER_FLYWHEEL = 56;

		public static final int COLLECTOR_TILT = 57;
		public static final int COLLECTOR_INTAKE = 58;
		public static final int CONVEYOR = 59;
	}

	/**
	 * Constants class for the ShooterBase subsystem and subcomponents. Contains lookup table for RPM/ToF/distance
	 */
	public static final class ShooterConstants {

		public static final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<>(
		  InverseInterpolator.forDouble(),

		  // Value interpolator: blends RPMs and flight times based on distance ratio, t
		  (start, end, t) -> new ShooterParams(
			MathUtil.interpolate(start.rpm(), end.rpm(), t),
			MathUtil.interpolate(start.timeOfFlight(), end.timeOfFlight(), t)
		  )
		);
		// Error to tolerate when spinning up to shoot (in RPMs)
		public static final double SPINUP_TOLERANCE_RPM = 50;
		// Speed [-1,1] to run the indexer at while shooting
		public static final double INDEXER_SPEED_WHILE_SHOOTING = 0.65;
		// Speed [-1,1] to run the conveyor at while shooting
		public static final double CONVEYOR_SPEED_WHILE_SHOOTING = 0.5;
		public static final double HUB_HEIGHT_METERS = 1.83; // End height of trajectory
		public static final double SHOOTER_HEIGHT_METERS = 0.5; // Start height of trajectory
		public static final double FUEL_EXIT_ANGLE_RADIANS = 0.983936078; // RADIANS At what angle does fuel leave the shooter

		/*
		 * Units of Flywheel Constants (Thanks Rev for good docs this year)
		 * - kP: Duty cycle per rotation of error
		 * - kI: Duty cycle per (rotation * ms)
		 * - kD: (Duty cycle * ms) per duty cycle
		 * - kS: Volts
		 * - kV: Volts per RPM
		 * - kA: Volts per RPM/sec
		 */

		// LEFT SHOOTER CONSTANTS
		public static final boolean LEFT_FLYWHEEL_INVERTED = true;
		public static final boolean LEFT_INDEXER_INVERTED = true;
		public static final double LEFT_INDEXER_P = 0.0;
		public static final double LEFT_INDEXER_I = 0.0;
		public static final double LEFT_INDEXER_D = 0.0;
		public static final double LEFT_INDEXER_S = 0.079892;
		public static final double LEFT_INDEXER_V = 0.10559 / 60;
		public static final double LEFT_INDEXER_A = 0.0073533 / 60;
		public static final double LEFT_FLYWHEEL_P = 0.001;
		public static final double LEFT_FLYWHEEL_I = 0.0;
		public static final double LEFT_FLYWHEEL_D = 0.01;
		public static final double LEFT_FLYWHEEL_S = 0.089332;
		public static final double LEFT_FLYWHEEL_V = 0.10669 / 60;
		public static final double LEFT_FLYWHEEL_A = 0.020699 / 60;
		// RIGHT SHOOTER CONSTANTS
		public static final boolean RIGHT_FLYWHEEL_INVERTED = false;
		public static boolean RIGHT_INDEXER_INVERTED = false;
		public static final double RIGHT_INDEXER_P = 0.0;
		public static final double RIGHT_INDEXER_I = 0.0;
		public static final double RIGHT_INDEXER_D = 0.0;
		public static final double RIGHT_INDEXER_S = 0.080536;
		public static final double RIGHT_INDEXER_V = 0.10572 / 60;
		public static final double RIGHT_INDEXER_A = 0.008408 / 60;
		public static final double RIGHT_FLYWHEEL_P = 0.001;
		public static final double RIGHT_FLYWHEEL_I = 0.0;
		public static final double RIGHT_FLYWHEEL_D = 0.01;
		public static final double RIGHT_FLYWHEEL_S = 0.04773;
		public static final double RIGHT_FLYWHEEL_V = 0.10424 / 60;
		public static final double RIGHT_FLYWHEEL_A = 0.020809 / 60;

		// For SOTF
		public static final double SYSTEM_LATENCY_SECONDS = 0.3;

		static {
			SHOOTER_MAP.put(2.6, new ShooterParams(2750, 0.68));
			SHOOTER_MAP.put(3.0, new ShooterParams(2850, 0.8));
			SHOOTER_MAP.put(3.5, new ShooterParams(2950, 0.9));
			SHOOTER_MAP.put(4.0, new ShooterParams(3100, 1.0));
			SHOOTER_MAP.put(4.865, new ShooterParams(3550, 1.3));
			SHOOTER_MAP.put(5.269, new ShooterParams(3800, 1.34));
		}

		public record ShooterParams(double rpm, double timeOfFlight) {}
	}

	/**
	 * Constants class for the Collector subsystem.
	 */
	public static final class CollectorConstants {
		public static final double TILT_P = 0.6;
		public static final double TILT_I = 0;
		public static final double TILT_D = 0;
		public static final double TILT_COS = 0.35487;
		public static final double TILT_S = 0.49117;
		public static final double TILT_V = 1.234 / 60;
		public static final double TILT_A = 0;

		// Error in rotations at which to stop tilting commands
		public static final double TILT_TOLERANCE_ROTATIONS = 0.05;

		public static final double THRUBORE_OFFSET = 0.91;

		public static final double THRUBORE_OUT = 0;
		public static final double THRUBORE_IN = 0.25;

		public static final double INTAKING_ROLLER_SPEED = 0.75;
		public static final double OUTTAKING_ROLLER_SPEED = -0.75;
		public static final double INTAKING_CONVEYOR_SPEED = 0.25;
	}
}