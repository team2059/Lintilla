// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla;

import static org.team2059.Lintilla.Constants.OperatorConstants.SHOOTER_ADD5PERCENT_SWITCH;
import static org.team2059.Lintilla.Constants.OperatorConstants.SHOOTER_SUB5PERCENT_SWITCH;
import static org.team2059.Lintilla.Constants.OperatorConstants.USE_XBOX_CONTROLLER;

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

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Central initialization class
 */
public class RobotContainer {

	public static Joystick logitech;
	public static CommandXboxController controller;
	public static GenericHID buttonBox;

	SendableChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// Initialize all controllers and button boxes
		if (OperatorConstants.USE_XBOX_CONTROLLER) {
			controller = new CommandXboxController(OperatorConstants.XBOX_PORT);
		} else {
			logitech = new Joystick(OperatorConstants.LOGITECH_PORT);
		}
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
		  new VortexShooter( // RIGHT SHOOTER
			CANConstants.LEFT_SHOOTER_FLYWHEEL,
			CANConstants.LEFT_SHOOTER_INDEXER,
			CANConstants.RIGHT_SHOOTER_FLYWHEEL,
			CANConstants.RIGHT_SHOOTER_INDEXER,
			ShooterConstants.FLYWHEEL_INVERTED,
			ShooterConstants.INDEXER_INVERTED,
			ShooterConstants.FLYWHEEL_P,
			ShooterConstants.FLYWHEEL_I,
			ShooterConstants.FLYWHEEL_D,
			ShooterConstants.FLYWHEEL_S,
			ShooterConstants.FLYWHEEL_V,
			ShooterConstants.FLYWHEEL_A,
			ShooterConstants.INDEXER_P,
			ShooterConstants.INDEXER_I,
			ShooterConstants.INDEXER_D,
			ShooterConstants.INDEXER_S,
			ShooterConstants.INDEXER_V,
			ShooterConstants.INDEXER_A
		  )
		);

		// Set the drivetrain's default command as the actual teleop command

		if (OperatorConstants.USE_XBOX_CONTROLLER) {
			Drivetrain.getInstance().setDefaultCommand(
			  new TeleopDriveCommand(
				Drivetrain.getInstance(),
				() -> -controller.getHID().getRawAxis(1), //left joystick y
        		() -> -controller.getHID().getRawAxis(0), //left joystick x 
        		() -> -controller.getHID().getRawAxis(4), //right joystick x
				() -> controller.getHID().getRawAxis(3), // slider 
				() -> false, // Strafe only (false)
				() -> false, // Inverted (false)
				() -> controller.getHID().getRawButton(6), // hub tracking 
				() -> false, // snake mode (false)
				() -> true
			  )
			);

		} else {
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
				() -> logitech.getRawButton(OperatorConstants.SNAKE_MODE),
				() -> false
			)
		);
		}
		

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

		// new JoystickButton(logitech, 7)
		// 	.whileTrue(ShooterBase.getInstance().indexerDynamicForward());

		// new JoystickButton(logitech, 8)
		// 	.whileTrue(ShooterBase.getInstance().indexerDynamicReverse());

		// new JoystickButton(logitech, 9)
		// 	.whileTrue(ShooterBase.getInstance().indexerQuasiForward());

		// new JoystickButton(logitech, 10)
		// 	.whileTrue(ShooterBase.getInstance().indexerQuasiReverse());

		/* RESET GYRO HEADING */
		if (USE_XBOX_CONTROLLER) {
			new JoystickButton(controller.getHID(), 4)
			  .whileTrue(Drivetrain.getInstance().resetGyroHeading());
		} else {
			new JoystickButton(logitech, OperatorConstants.RESET_HEADING)
		  		.whileTrue(Drivetrain.getInstance().resetGyroHeading());
		}
		
		/* SWITCH FIELD/ROBOT RELATIVITY */
		if (USE_XBOX_CONTROLLER) {
			new JoystickButton(controller.getHID(), 3)
		  		.whileTrue(Drivetrain.getInstance().setFieldRelativity());
		} else {
			new JoystickButton(logitech, OperatorConstants.ROBOT_RELATIVE)
		  		.whileTrue(Drivetrain.getInstance().setFieldRelativity());
		}
		
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
			  2000
			).alongWith(Collector.getInstance().agitationCommand())
		  );

		/* SHOOTER & HOPPER UNJAM */
		/* Runs both the shooter indexers and the conveyor in the reverse direction. */
		/* Intention is for this to be held for less than a second to loosen up a thick stack of fuel in the hopper. */
		new JoystickButton(buttonBox, OperatorConstants.SHOOTER_UNJAM)
		  .whileTrue(
			ShooterBase.getInstance().unjamShooters()
			  .alongWith(Conveyor.getInstance().conveyorOut())
		  );

		/* COLLECTOR OUT & INTAKE */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_OUT_INTAKE)
		  .whileTrue(
			Collector.getInstance().tiltOutAndIntake()
			  .alongWith(Conveyor.getInstance().conveyorIn())
		  );

		/* COLLECTOR TILT IN */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_IN)
		  .whileTrue(Collector.getInstance().tiltIn());

		/* COLLECTOR OUTTAKE/UNJAM */
		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_UNJAM)
		  .whileTrue(Collector.getInstance().outtake().alongWith(Conveyor.getInstance().conveyorOut()));

		// /* COLLECTOR ROLLERS IN/INTAKE at 25% */
		// new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_INTAKE)
		//   .whileTrue(Collector.getInstance().intakeSlow());

		new JoystickButton(buttonBox, OperatorConstants.COLLECTOR_INTAKE)
		  .whileTrue(Conveyor.getInstance().conveyorIn().alongWith(Collector.getInstance().intake()));

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
