// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team2059.Lintilla.Constants.CANConstants;
import org.team2059.Lintilla.Constants.DrivetrainConstants;
import org.team2059.Lintilla.Constants.OperatorConstants;
import org.team2059.Lintilla.Constants.ShooterConstants;
import org.team2059.Lintilla.commands.SpinupAndShootCommand;
import org.team2059.Lintilla.commands.TeleopDriveCommand;
import org.team2059.Lintilla.subsystems.vision.LocalizationSystem;
import org.team2059.Lintilla.subsystems.collector.Collector;
import org.team2059.Lintilla.subsystems.collector.CollectorIOReal;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.drivetrain.MK5nModule;
import org.team2059.Lintilla.subsystems.drivetrain.Pigeon2Gyroscope;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;
import org.team2059.Lintilla.subsystems.shooter.VortexShooter;

import static org.team2059.Lintilla.Constants.OperatorConstants.SHOOTER_ADDTENPERCENT_SWITCH;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	public static Joystick logitech;
	public static GenericHID buttonBox;

	public static Drivetrain drivetrain;
	public static LocalizationSystem localizationSystem;
	public static ShooterBase shooterBase;
	public static Collector collector;

	SendableChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		/* =========== */
		/* CONTROLLERS */
		/* =========== */

		logitech = new Joystick(OperatorConstants.LOGITECH_PORT);
		buttonBox = new GenericHID(OperatorConstants.BUTTON_BOX_PORT);

		/* ========== */
		/* SUBSYSTEMS */
		/* ========== */
		drivetrain = new Drivetrain(
		  new Pigeon2Gyroscope(CANConstants.PIGEON, CANConstants.CANIVORE),
		  new MK5nModule(
			CANConstants.FL_DRIVE,
			CANConstants.FL_TURN,
			CANConstants.FL_CANCODER,
			DrivetrainConstants.FL_ENCODER_OFFSET,
			DrivetrainConstants.FL_INVERTED
		  ),
		  new MK5nModule(
			CANConstants.FR_DRIVE,
			CANConstants.FR_TURN,
			CANConstants.FR_CANCODER,
			DrivetrainConstants.FR_ENCODER_OFFFSET,
			DrivetrainConstants.FR_INVERTED
		  ),
		  new MK5nModule(
			CANConstants.BL_DRIVE,
			CANConstants.BL_TURN,
			CANConstants.BL_CANCODER,
			DrivetrainConstants.BL_ENCODER_OFFSET,
			DrivetrainConstants.BL_INVERTED
		  ),
		  new MK5nModule(
			CANConstants.BR_DRIVE,
			CANConstants.BR_TURN,
			CANConstants.BR_CANCODER,
			DrivetrainConstants.BR_ENCODER_OFFSET,
			DrivetrainConstants.BR_INVERTED
		  )
		);

		shooterBase = new ShooterBase(
		  new VortexShooter( // LEFT SHOOTER
			CANConstants.LEFT_SHOOTER_FLYWHEEL,
			CANConstants.LEFT_SHOOTER_INDEXER,
			ShooterConstants.LEFT_FLYWHEEL_INVERTED,
			ShooterConstants.LEFT_INDEXER_INVERTED,
			ShooterConstants.LEFT_FLYWHEEL_P,
			ShooterConstants.LEFT_FLYWHEEL_I,
			ShooterConstants.LEFT_FLYWHEEL_D,
			ShooterConstants.LEFT_FLYWHEEL_S,
			ShooterConstants.LEFT_FLYWHEEL_V,
			ShooterConstants.LEFT_FLYWHEEL_A,
			ShooterConstants.LEFT_INDEXER_P,
			ShooterConstants.LEFT_INDEXER_I,
			ShooterConstants.LEFT_INDEXER_D,
			ShooterConstants.LEFT_INDEXER_S,
			ShooterConstants.LEFT_INDEXER_V,
			ShooterConstants.LEFT_INDEXER_A
		  ),
		  new VortexShooter( // RIGHT SHOOTER
			CANConstants.RIGHT_SHOOTER_FLYWHEEL,
			CANConstants.RIGHT_SHOOTER_INDEXER,
			ShooterConstants.RIGHT_FLYWHEEL_INVERTED,
			ShooterConstants.RIGHT_INDEXER_INVERTED,
			ShooterConstants.RIGHT_FLYWHEEL_P,
			ShooterConstants.RIGHT_FLYWHEEL_I,
			ShooterConstants.RIGHT_FLYWHEEL_D,
			ShooterConstants.RIGHT_FLYWHEEL_S,
			ShooterConstants.RIGHT_FLYWHEEL_V,
			ShooterConstants.RIGHT_FLYWHEEL_A,
			ShooterConstants.RIGHT_INDEXER_P,
			ShooterConstants.RIGHT_INDEXER_I,
			ShooterConstants.RIGHT_INDEXER_D,
			ShooterConstants.RIGHT_INDEXER_S,
			ShooterConstants.RIGHT_INDEXER_V,
			ShooterConstants.RIGHT_INDEXER_A
		  )
		);

		drivetrain.setDefaultCommand(
		  new TeleopDriveCommand(
			drivetrain,
			shooterBase,
			() -> -logitech.getRawAxis(OperatorConstants.TRANSLATION_AXIS), // forwardX
			() -> -logitech.getRawAxis(OperatorConstants.STRAFE_AXIS), // forwardY
			() -> -logitech.getRawAxis(OperatorConstants.ROTATION_AXIS), // rotation
			() -> logitech.getRawAxis(OperatorConstants.SLIDER_AXIS), // slider
			() -> logitech.getRawButton(OperatorConstants.STRAFE_ONLY), // Strafe Only Button
			() -> logitech.getRawButton(OperatorConstants.INVERT_DRIVE), // Inverted button
			() -> logitech.getRawButton(OperatorConstants.HUB_ALIGN)
		  )
		);

		localizationSystem = new LocalizationSystem();

		collector = new Collector(
		  new CollectorIOReal(
			CANConstants.COLLECTOR_TILT,
			CANConstants.COLLECTOR_INTAKE,
			CANConstants.CONVEYOR
		  )
		);

		/* ========== */
		/* AUTONOMOUS */
		/* ========== */

		/* NAMED COMMANDS */
		NamedCommands.registerCommand(
		  "ShootWithDistance",
		  new SpinupAndShootCommand(drivetrain, shooterBase, collector)
		);

		NamedCommands.registerCommand(
		  "CollectorOutInfinite",
		  collector.tiltOutInfinite()
		);

		NamedCommands.registerCommand(
		  "CollectorInInfinite",
		  collector.tiltInInfinite()
		);

		NamedCommands.registerCommand(
		  "CollectorOut",
		  collector.tiltOut()
		);

		NamedCommands.registerCommand(
		  "CollectorIn",
		  collector.tiltIn()
		);

		NamedCommands.registerCommand(
		  "Intake",
		  collector.intake()
		);

		// Build auto chooser - you can also set a default.
		autoChooser = AutoBuilder.buildAutoChooser();

		// Publish auto chooser
		SmartDashboard.putData("Auto Chooser", autoChooser);

		/* ======= */
		/* LOGGING */
		/* ======= */

		// Allow viewing of command scheduler queue in dashboards
		SmartDashboard.putData(CommandScheduler.getInstance());

		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {

		/* =================== */
		/* DRIVER'S CONTROLLER */
		/* =================== */

		/* RESET GYRO HEADING */
		new JoystickButton(logitech, OperatorConstants.RESET_HEADING)
		  .whileTrue(
			Commands.runOnce(() -> drivetrain.resetGyroHeading())
			  .ignoringDisable(true)
		  );

		/* SWITCH FIELD/ROBOT RELATIVITY */
		new JoystickButton(logitech, OperatorConstants.ROBOT_RELATIVE)
		  .whileTrue(
			Commands.runOnce(() -> drivetrain.setFieldRelativity())
			  .ignoringDisable(true)
		  );

		/* ===================== */
		/* OPERATOR'S CONTROLLER */
		/* ===================== */

		/* SPINUP & SHOOT FROM CURRENT HUB DISTANCE */
		new JoystickButton(buttonBox, OperatorConstants.SPINUP_SHOOT_DISTANCE)
		  .whileTrue(
			new SpinupAndShootCommand(
			  drivetrain,
			  shooterBase,
			  collector
			)
		  );

		/* SPINUP & SHOOT WITH FIXED RPM */
		new JoystickButton(buttonBox, OperatorConstants.SPINUP_SHOOT_FIXED)
		  .whileTrue(
			new SpinupAndShootCommand(
			  drivetrain,
			  shooterBase,
			  collector,
			  2800
			)
		  );

		new JoystickButton(buttonBox, 3)
		  .whileTrue(
			Commands.startEnd(
			  () -> {
				  shooterBase.leftShooter.setIndexerSpeed(-1);
				  shooterBase.rightShooter.setIndexerSpeed(-1);
			  },
			  () -> {
				  shooterBase.stopAllSubsystemMotors();
			  }
			)
		  );

		/* COLLECTOR OUT & INTAKE */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_OUT_INTAKE)
		  .whileTrue(collector.tiltOutAndIntake());

		/* COLLECTOR TILT IN */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_IN)
		  .whileTrue(collector.tiltIn());

		/* COLLECTOR OUTTAKE/UNJAM */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_UNJAM)
		  .whileTrue(collector.outtake());

		/* COLLECTOR ROLLERS IN/INTAKE */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_INTAKE)
		  .whileTrue(collector.intake());

		/* QUEST MEASUREMENTS SWITCH */
		new JoystickButton(buttonBox, OperatorConstants.QUEST_MEASUREMENT_SWITCH)
		  .onFalse(localizationSystem.enableQnavMeasurements())
		  .onTrue(localizationSystem.disableQnavMeasurements());

		/* PHOTONVISION MEASUREMENTS SWITCH */
		new JoystickButton(buttonBox, OperatorConstants.PHOTONVISION_MEASUREMENT_SWITCH)
		  .onFalse(localizationSystem.enablePVMeasurements())
		  .onTrue(localizationSystem.disablePVMeasurements());

		new JoystickButton(buttonBox, SHOOTER_ADDTENPERCENT_SWITCH)
		  .onFalse(Commands.runOnce(() -> {
			  shooterBase.setAddFivePercent(true);
		  })
			  .ignoringDisable(true)
		  ).onTrue(Commands.runOnce(() -> {
			  shooterBase.setAddFivePercent(false);
		  })
			  .ignoringDisable(true)
		  );

		new JoystickButton(buttonBox, 12)
		  .whileTrue(Commands.runOnce(() -> {
				  localizationSystem.syncPoses();
			  })
			  .ignoringDisable(true)
		  );
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
