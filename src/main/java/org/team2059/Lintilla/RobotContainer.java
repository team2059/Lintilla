// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.team2059.Lintilla.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team2059.Lintilla.commands.TeleopDriveCmd;
import org.team2059.Lintilla.subsystems.drivetrain.Drivetrain;
import org.team2059.Lintilla.subsystems.drivetrain.MK5nModule;
import org.team2059.Lintilla.subsystems.drivetrain.Pigeon2Gyroscope;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    SendableChooser<Command> autoChooser;

    public static Joystick logitech;

    public static Drivetrain drivetrain;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /* ========== */
        /* SUBSYSTEMS */
        /* ========== */
        drivetrain = new Drivetrain(
          new Pigeon2Gyroscope(CANConstants.pigeon2), // (, CANConstants.canivore),
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

        /* =========== */
        /* CONTROLLERS */
        /* =========== */

        logitech = new Joystick(OperatorConstants.logitechPort);

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
