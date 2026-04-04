// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.team2059.Lintilla.Constants.CANConstants;
import org.team2059.Lintilla.Constants.DrivetrainConstants;
import org.team2059.Lintilla.Constants.OperatorConstants;
import org.team2059.Lintilla.Constants.ShooterConstants;
import org.team2059.Lintilla.commands.SpinupAndShootCommand;
import org.team2059.Lintilla.commands.TeleopDriveCommand;
import org.team2059.Lintilla.subsystems.collector.Collector;
import org.team2059.Lintilla.subsystems.collector.CollectorIOReal;
import org.team2059.Lintilla.subsystems.conveyor.Conveyor;
import org.team2059.Lintilla.subsystems.conveyor.ConveyorIOReal;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.drivetrain.MK5nModule;
import org.team2059.Lintilla.subsystems.drivetrain.Pigeon2Gyroscope;
import org.team2059.Lintilla.subsystems.shooter.ShooterBase;
import org.team2059.Lintilla.subsystems.shooter.VortexShooter;
import org.team2059.Lintilla.subsystems.vision.LocalizationSystem;

import static org.team2059.Lintilla.Constants.OperatorConstants.SHOOTER_ADD5PERCENT_SWITCH;
import static org.team2059.Lintilla.Constants.OperatorConstants.SHOOTER_SUB5PERCENT_SWITCH;

/**
 * Central initialization class
 */
public class RobotContainer {

	public static Joystick logitech;
	public static GenericHID buttonBox;

	SendableChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// Initialize all controllers and button boxes

		logitech = new Joystick(OperatorConstants.LOGITECH_PORT);
		buttonBox = new GenericHID(OperatorConstants.BUTTON_BOX_PORT);

		// Initialize all instance objects for subsystems and default commands

		Drivetrain.initialize(
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

		ShooterBase.initialize(
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

		// Set the drivetrain's default command as the actual teleop command
		Drivetrain.getInstance().setDefaultCommand(
		  new TeleopDriveCommand(
			Drivetrain.getInstance(),
			() -> -logitech.getRawAxis(OperatorConstants.TRANSLATION_AXIS), // forwardX
			() -> -logitech.getRawAxis(OperatorConstants.STRAFE_AXIS), // forwardY
			() -> -logitech.getRawAxis(OperatorConstants.ROTATION_AXIS), // rotation
			() -> logitech.getRawAxis(OperatorConstants.SLIDER_AXIS), // slider
			() -> logitech.getRawButton(OperatorConstants.STRAFE_ONLY), // Strafe Only Button
			() -> logitech.getRawButton(OperatorConstants.INVERT_DRIVE), // Inverted button
			() -> logitech.getRawButton(OperatorConstants.HUB_ALIGN),
			() -> logitech.getRawButton(OperatorConstants.SNAKE_MODE)
		  )
		);

		LocalizationSystem.initialize();

		Collector.initialize(
		  new CollectorIOReal(
			CANConstants.COLLECTOR_TILT,
			CANConstants.COLLECTOR_INTAKE_LEFT,
		    CANConstants.COLLECTOR_INTAKE_RIGHT
		  )
		);

		Conveyor.initialize(
		  new ConveyorIOReal(
			CANConstants.CONVEYOR
		  )
		);


		/* ========== */
		/* AUTONOMOUS */
		/* ========== */

		// Register NamedCommands
		Autos.registerNamedCommands();

		// Build auto chooser - you can also set a default.
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		/* ======= */
		/* LOGGING */
		/* ======= */

		// Allow viewing of command scheduler queue in dashboards
		SmartDashboard.putData(CommandScheduler.getInstance());

		configureBindings();
	}

	/**
	 * Maps buttons to specific commands.
	 * <p>
	 * Important to note that this method runs ONCE. The commands are not remade, they are initialized once and saved.
	 */
	private void configureBindings() {

		/* =================== */
		/* DRIVER'S CONTROLLER */
		/* =================== */

		/* RESET GYRO HEADING */
		new JoystickButton(logitech, OperatorConstants.RESET_HEADING)
		  .whileTrue(Drivetrain.getInstance().resetGyroHeading());

		/* SWITCH FIELD/ROBOT RELATIVITY */
		new JoystickButton(logitech, OperatorConstants.ROBOT_RELATIVE)
		  .whileTrue(Drivetrain.getInstance().setFieldRelativity());

		/* ===================== */
		/* OPERATOR'S CONTROLLER */
		/* ===================== */

		/* SPINUP & SHOOT FROM CURRENT HUB DISTANCE */
		new JoystickButton(buttonBox, OperatorConstants.SPINUP_SHOOT_DISTANCE)
		  .whileTrue(
			new SpinupAndShootCommand(
			  ShooterBase.getInstance(),
			  Conveyor.getInstance()
			).alongWith(Collector.getInstance().agitationCommand())
		  );

		/* SPINUP & SHOOT WITH FIXED RPM */
		new JoystickButton(buttonBox, OperatorConstants.SPINUP_SHOOT_FIXED)
		  .whileTrue(
			new SpinupAndShootCommand(
			  ShooterBase.getInstance(),
			  Conveyor.getInstance(),
			  2800
			).alongWith(Collector.getInstance().agitationCommand())
		  );

		/* SHOOTER UNJAM */
		new JoystickButton(buttonBox, OperatorConstants.SHOOTER_UNJAM)
		  .whileTrue(ShooterBase.getInstance().unjamShooters());

		/* COLLECTOR OUT & INTAKE */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_OUT_INTAKE)
		  .whileTrue(Collector.getInstance().tiltOutAndIntake());

		/* COLLECTOR TILT IN */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_IN)
		  .whileTrue(Collector.getInstance().tiltIn());

		/* COLLECTOR OUTTAKE/UNJAM */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_UNJAM)
		  .whileTrue(Collector.getInstance().outtake());

		/* COLLECTOR ROLLERS IN/INTAKE */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_INTAKE)
		  .whileTrue(Collector.getInstance().intake());

		/* QUEST MEASUREMENTS SWITCH */
		new JoystickButton(buttonBox, OperatorConstants.QUEST_MEASUREMENT_SWITCH)
		  .onFalse(LocalizationSystem.getInstance().enableQnavMeasurements())
		  .onTrue(LocalizationSystem.getInstance().disableQnavMeasurements());

		/* PHOTONVISION MEASUREMENTS SWITCH */
		new JoystickButton(buttonBox, OperatorConstants.PHOTONVISION_MEASUREMENT_SWITCH)
		  .onFalse(LocalizationSystem.getInstance().enablePVMeasurements())
		  .onTrue(LocalizationSystem.getInstance().disablePVMeasurements());

		/* ADD 5% TO ALL RPM OUTPUTS */
		new JoystickButton(buttonBox, SHOOTER_ADD5PERCENT_SWITCH)
		  .onFalse(Commands.runOnce(() -> {
				  ShooterBase.getInstance().setAddFivePercent(true);
			  })
			  .ignoringDisable(true)
		  ).onTrue(Commands.runOnce(() -> {
				  ShooterBase.getInstance().setAddFivePercent(false);
			  })
			  .ignoringDisable(true)
		  );

		/* SUBTRACT 5% FROM ALL RPM OUTPUTS */
		new JoystickButton(buttonBox, SHOOTER_SUB5PERCENT_SWITCH)
		  .onFalse(Commands.runOnce(() -> {
				  ShooterBase.getInstance().setSubFivePercent(true);
			  })
			  .ignoringDisable(true)
		  ).onTrue(Commands.runOnce(() -> {
				  ShooterBase.getInstance().setSubFivePercent(false);
			  })
			  .ignoringDisable(true)
		  );

		/* SYNC PHOTONVISION AND QUEST POSES */
		new JoystickButton(buttonBox, OperatorConstants.LOCALIZATION_SYNC_POSES)
		  .whileTrue(LocalizationSystem.getInstance().syncPoses());
	}

	/**
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
