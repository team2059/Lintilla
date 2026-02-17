// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team2059.Lintilla.Constants.CANConstants;
import org.team2059.Lintilla.Constants.DrivetrainConstants;
import org.team2059.Lintilla.Constants.OperatorConstants;
import org.team2059.Lintilla.Constants.ShooterConstants;
import org.team2059.Lintilla.commands.SpinupAndShootCmd;
import org.team2059.Lintilla.commands.TeleopDriveCmd;
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
	public static ShooterBase shooterBase;
	public static Collector collector;

	SendableChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

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

		collector = new Collector(
		  new CollectorIOReal(
			CANConstants.collectorTiltMotor,
			CANConstants.collectorIntakeMotor,
			CANConstants.conveyorMotor
		  )
		);

		/* =========== */
		/* CONTROLLERS */
		/* =========== */

		logitech = new Joystick(OperatorConstants.logitechPort);

		buttonBox = new GenericHID(OperatorConstants.buttonBoxPort);

		drivetrain.setDefaultCommand(
		  new TeleopDriveCmd(
			drivetrain,
			() -> -logitech.getRawAxis(OperatorConstants.JoystickTranslationAxis), // forwardX
			() -> -logitech.getRawAxis(OperatorConstants.JoystickStrafeAxis), // forwardY
			() -> -logitech.getRawAxis(OperatorConstants.JoystickRotationAxis), // rotation
			() -> logitech.getRawAxis(OperatorConstants.JoystickSliderAxis), // slider
			() -> logitech.getRawButton(OperatorConstants.JoystickStrafeOnly), // Strafe Only Button
			() -> logitech.getRawButton(OperatorConstants.JoystickInvertedDrive) // Inverted button
		  )
		);

		/* ========== */
		/* AUTONOMOUS */
		/* ========== */

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
		/* RESET GYRO HEADING */
		new JoystickButton(logitech, OperatorConstants.JoystickResetHeading)
		  .whileTrue(new InstantCommand(() -> drivetrain.resetGyroHeading()));

		/* SWITCH FIELD/ROBOT RELATIVITY IN TELEOP */
		new JoystickButton(logitech, OperatorConstants.JoystickRobotRelative)
		  .whileTrue(new InstantCommand(() -> drivetrain.setFieldRelativity()));

    // new JoystickButton(buttonBox, 1)
	// 	  .whileTrue(shooterBase.leftShooterQuasiForward());

	// new JoystickButton(buttonBox, 2)
	// 	.whileTrue(shooterBase.leftShooterQuasiReverse());

	// new JoystickButton(buttonBox, 3)
	// 	.whileTrue(shooterBase.leftShooterDynamicForward());

	// new JoystickButton(buttonBox, 4)
	// 	.whileTrue(shooterBase.leftShooterDynamicReverse());

	// new JoystickButton(buttonBox, 5)
	// 	.whileTrue(shooterBase.rightShooterQuasiForward());

	// new JoystickButton(buttonBox, 6)
	// 	.whileTrue(shooterBase.rightShooterQuasiReverse());

	// new JoystickButton(buttonBox, 7)
	// 	.whileTrue(shooterBase.rightShooterDynamicForward());

	// new JoystickButton(buttonBox, 8)
	// 	.whileTrue(shooterBase.rightShooterDynamicReverse());

	/* Indexer SysID
    new JoystickButton(buttonBox, 1)
		  .whileTrue(shooterBase.leftIndexerQuasiForward());

    new JoystickButton(buttonBox, 2)
		  .whileTrue(shooterBase.leftIndexerQuasiReverse());

    new JoystickButton(buttonBox, 3)
		  .whileTrue(shooterBase.leftIndexerDynamicForward());

    new JoystickButton(buttonBox, 4)
		  .whileTrue(shooterBase.leftIndexerDynamicReverse());

    new JoystickButton(buttonBox, 5)
		  .whileTrue(shooterBase.rightIndexerQuasiForward());

    new JoystickButton(buttonBox, 6)
		  .whileTrue(shooterBase.rightIndexerQuasiReverse());

    new JoystickButton(buttonBox, 7)
		  .whileTrue(shooterBase.rightIndexerDynamicForward());

    new JoystickButton(buttonBox, 8)
		  .whileTrue(shooterBase.rightIndexerDynamicReverse());
 	*/

	/* Drivetrain translation sysID routine (just drive motors) (wheels must be locked straight for this)

       new JoystickButton(buttonBox, 1)
        .whileTrue(drivetrain.routine.quasistaticForward());
       new JoystickButton(buttonBox, 2)
         .whileTrue(drivetrain.routine.quasistaticReverse());
       new JoystickButton(buttonBox, 3)
        .whileTrue(drivetrain.routine.dynamicForward());
       new JoystickButton(buttonBox, 4)
        .whileTrue(drivetrain.routine.dynamicReverse());
	 */
	
	new JoystickButton(buttonBox, 1)
		.whileTrue(new SpinupAndShootCmd(shooterBase, collector, 1500));

	new JoystickButton(buttonBox, 2)
	  .whileTrue(new InstantCommand(() -> collector.runIntake(0.5)))
	  .onFalse(new InstantCommand(() -> collector.stopIntake()));

	new JoystickButton(buttonBox, 5)
	  .whileTrue(collector.collectorOut())
	  .onFalse(new InstantCommand(() -> collector.io.stopTilt()));

	new JoystickButton(buttonBox, 6)
	  .whileTrue(collector.collectorIn())
	  .onFalse(new InstantCommand(() -> collector.io.stopTilt()));

	new JoystickButton(buttonBox, 9)
      .whileTrue(new InstantCommand(() -> {
		  drivetrain.setQuestRobotPose(Pose3d.kZero);
      }));

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
