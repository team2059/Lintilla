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

		logitech = new Joystick(OperatorConstants.logitechPort);
		buttonBox = new GenericHID(OperatorConstants.buttonBoxPort);

		/* ========== */
		/* SUBSYSTEMS */
		/* ========== */
		drivetrain = new Drivetrain(
		  new Pigeon2Gyroscope(CANConstants.pigeon2, CANConstants.canivore),
		  new MK5nModule(
			CANConstants.frontLeftDriveMotor,
			CANConstants.frontLeftTurnMotor,
			CANConstants.frontLeftCancoder,
			DrivetrainConstants.frontLeftEncoderOffset,
			DrivetrainConstants.frontLeftInverted
		  ),
		  new MK5nModule(
			CANConstants.frontRightDriveMotor,
			CANConstants.frontRightTurnMotor,
			CANConstants.frontRightCancoder,
			DrivetrainConstants.frontRightEncoderOffset,
			DrivetrainConstants.frontRightInverted
		  ),
		  new MK5nModule(
			CANConstants.backLeftDriveMotor,
			CANConstants.backLeftTurnMotor,
			CANConstants.backLeftCancoder,
			DrivetrainConstants.backLeftEncoderOffset,
			DrivetrainConstants.backLeftInverted
		  ),
		  new MK5nModule(
			CANConstants.backRightDriveMotor,
			CANConstants.backRightTurnMotor,
			CANConstants.backRightCancoder,
			DrivetrainConstants.backRightEncoderOffset,
			DrivetrainConstants.backRightInverted
		  )
		);

		shooterBase = new ShooterBase(
		  new VortexShooter( // LEFT SHOOTER
			CANConstants.leftShooterFlywheel,
			CANConstants.leftShooterIndexerMotor,
			ShooterConstants.leftFlywheelInverted,
			ShooterConstants.leftIndexerInverted,
			ShooterConstants.leftFlywheelkP,
			ShooterConstants.leftFlywheelkI,
			ShooterConstants.leftFlywheelkD,
			ShooterConstants.leftFlywheelkS,
			ShooterConstants.leftFlywheelkV,
			ShooterConstants.leftFlywheelkA,
			ShooterConstants.leftIndexerkP,
			ShooterConstants.leftIndexerkI,
			ShooterConstants.leftIndexerkD,
			ShooterConstants.leftIndexerkS,
			ShooterConstants.leftIndexerkV,
			ShooterConstants.leftIndexerkA
		  ),
		  new VortexShooter( // RIGHT SHOOTER
			CANConstants.rightShooterFlywheel,
			CANConstants.rightShooterIndexerMotor,
			ShooterConstants.rightFlywheelInverted,
			ShooterConstants.rightIndexerInverted,
			ShooterConstants.rightFlywheelkP,
			ShooterConstants.rightFlywheelkI,
			ShooterConstants.rightFlywheelkD,
			ShooterConstants.rightFlywheelkS,
			ShooterConstants.rightFlywheelkV,
			ShooterConstants.rightFlywheelkA,
			ShooterConstants.rightIndexerkP,
			ShooterConstants.rightIndexerkI,
			ShooterConstants.rightIndexerkD,
			ShooterConstants.rightIndexerkS,
			ShooterConstants.rightIndexerkV,
			ShooterConstants.rightIndexerkA
		  )
		);

		drivetrain.setDefaultCommand(
		  new TeleopDriveCommand(
			drivetrain,
			shooterBase,
			() -> -logitech.getRawAxis(OperatorConstants.JoystickTranslationAxis), // forwardX
			() -> -logitech.getRawAxis(OperatorConstants.JoystickStrafeAxis), // forwardY
			() -> -logitech.getRawAxis(OperatorConstants.JoystickRotationAxis), // rotation
			() -> logitech.getRawAxis(OperatorConstants.JoystickSliderAxis), // slider
			() -> logitech.getRawButton(OperatorConstants.JoystickStrafeOnly), // Strafe Only Button
			() -> logitech.getRawButton(OperatorConstants.JoystickInvertedDrive), // Inverted button
			() -> logitech.getRawButton(2)
		  )
		);

		localizationSystem = new LocalizationSystem();

		collector = new Collector(
		  new CollectorIOReal(
			CANConstants.collectorTiltMotor,
			CANConstants.collectorIntakeMotor,
			CANConstants.conveyorMotor
		  )
		);

		/* ========== */
		/* AUTONOMOUS */
		/* ========== */

		/* NAMED COMMANDS */

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
		new JoystickButton(logitech, OperatorConstants.JoystickResetHeading)
		  .whileTrue(
			Commands.runOnce(() -> drivetrain.resetGyroHeading())
			  .ignoringDisable(true)
		  );

		/* SWITCH FIELD/ROBOT RELATIVITY */
		new JoystickButton(logitech, OperatorConstants.JoystickRobotRelative)
		  .whileTrue(
			Commands.runOnce(() -> drivetrain.setFieldRelativity())
			  .ignoringDisable(true)
		  );

		/* ===================== */
		/* OPERATOR'S CONTROLLER */
		/* ===================== */

		/* SPINUP & SHOOT FROM CURRENT HUB DISTANCE */
		new JoystickButton(buttonBox, OperatorConstants.ButtonBoxSpinupShootDistance)
		  .whileTrue(
			new SpinupAndShootCommand(
			  drivetrain,
			  shooterBase,
			  collector
			)
		  );

		/* SPINUP & SHOOT WITH FIXED RPM */
		new JoystickButton(buttonBox, OperatorConstants.ButtonBoxSpinupShootFixed)
		  .whileTrue(
			new SpinupAndShootCommand(
			  drivetrain,
			  shooterBase,
			  collector,
			  2000
			)
		  );

		/* COLLECTOR OUT & INTAKE */
		new JoystickButton(buttonBox, OperatorConstants.ButtonBoxCollectorOutIntake)
		  .whileTrue(collector.tiltOutAndIntake());

		/* COLLECTOR TILT IN */
		new JoystickButton(buttonBox, OperatorConstants.ButtonBoxCollectorIn)
		  .whileTrue(collector.tiltIn());

		/* COLLECTOR OUTTAKE/UNJAM */
		new JoystickButton(buttonBox, OperatorConstants.ButtonBoxCollectorUnjam)
		  .whileTrue(collector.outtake());

		/* COLLECTOR ROLLERS IN/INTAKE */
		new JoystickButton(buttonBox, OperatorConstants.ButtonBoxCollectorIntake)
		  .whileTrue(collector.outtake());

		/* QUEST MEASUREMENTS SWITCH */
		new JoystickButton(buttonBox, OperatorConstants.ButtonBoxQuestMeasurement)
		  .onFalse(
			Commands.runOnce(() -> localizationSystem.setQnavUseMeasurements(true))
			  .ignoringDisable(true)
		  )
		  .onTrue(
			Commands.runOnce(() -> localizationSystem.setQnavUseMeasurements(false))
			  .ignoringDisable(true)
		  );

		/* PHOTONVISION MEASUREMENTS SWITCH */
		new JoystickButton(buttonBox, OperatorConstants.ButtonBoxPhotonVisionMeasurement)
		  .onFalse(
			Commands.runOnce(() -> localizationSystem.setPVUseMeasurements(true))
			  .ignoringDisable(true)
		  )
		  .onTrue(
			Commands.runOnce(() -> localizationSystem.setPVUseMeasurements(false))
			  .ignoringDisable(true)
		  );

		new JoystickButton(buttonBox, 11)
		  .whileTrue(collector.tiltOut());

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
