// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.Lintilla;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team2059.Lintilla.util.LocalADStarAK;

import java.util.Optional;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

	private final RobotContainer robotContainer;
	private Command autonomousCommand;

	private double lastLoopTime = 0.0;
	private int loopOverrunCount = 0;

	private boolean gameDataParsed = false;
	private boolean shift1Active = false;
	private double timeUntilHubActive = 0.0;
	private double timeUntilHubInactive = 0.0;

	/**
	 * This method is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {

		Pathfinding.setPathfinder(new LocalADStarAK());

		Logger.recordMetadata("ProjectName", "Lintilla");

		if (isReal()) {
			Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			LoggedPowerDistribution.getInstance(Constants.CANConstants.PDH, PowerDistribution.ModuleType.kRev);
		} else {
			setUseTiming(false); // Run as fast as possible
			Logger.addDataReceiver(new NT4Publisher());
			String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
		}

		Logger.start();

		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		robotContainer = new RobotContainer();

		// Schedule warmup commands for PathPlanner path finding and following
		CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
		CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
	}


	/**
	 * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic methods, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {

		double currentTime = Timer.getFPGATimestamp();
		double loopDurationSeconds = currentTime - lastLoopTime;

		if (lastLoopTime != 0.0 && loopDurationSeconds > 0.03) loopOverrunCount++;

		Logger.recordOutput("LoopDurationMs", loopDurationSeconds * 1000.0);
		Logger.recordOutput("LoopOverrunCount", loopOverrunCount);

		lastLoopTime = currentTime;

		boolean hubActive = isHubActive();
		Logger.recordOutput("HubActive", hubActive);
		Logger.recordOutput("TimeUntilHubActive", timeUntilHubActive);
		Logger.recordOutput("Time", DriverStation.getMatchTime());
		Logger.recordOutput("TimeUntilHubInactive", timeUntilHubInactive);
		Logger.recordOutput("Game Data Parsed", gameDataParsed);

		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}


	/**
	 * This method is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	public boolean isHubActive() {
		Optional<Alliance> alliance = DriverStation.getAlliance();

		if (!gameDataParsed) {
			String gameData = DriverStation.getGameSpecificMessage();
			if (alliance.isPresent() && !gameData.isEmpty()) {
				boolean redInactiveFirst = (gameData.charAt(0) == 'R');

				shift1Active = switch (alliance.get()) {
					case Red -> !redInactiveFirst;
					case Blue -> redInactiveFirst;
				};
				gameDataParsed = true;
			} else {
				timeUntilHubActive = 0;
				timeUntilHubInactive = 0;
				return true;
			}
		}

		if (DriverStation.isAutonomousEnabled()) {
			timeUntilHubActive = 0;
			timeUntilHubInactive = 0;
			return true;
		}

		if (!DriverStation.isTeleopEnabled()) {
			timeUntilHubActive = 0;
			timeUntilHubInactive = 0;
			return false;
		}

		double matchTime = DriverStation.getMatchTime();

		if (matchTime > 130) {
			timeUntilHubActive = 0;
			timeUntilHubInactive = shift1Active ? matchTime - 105 : matchTime - 130;
			return true;
		} else if (matchTime > 105) {
			timeUntilHubActive = shift1Active ? 0 : matchTime - 105;
			timeUntilHubInactive = shift1Active ? matchTime - 105 : 0;
			return shift1Active;
		} else if (matchTime > 80) {
			timeUntilHubActive = !shift1Active ? 0 : matchTime - 80;
			timeUntilHubInactive = !shift1Active ? matchTime - 80 : 0;
			return !shift1Active;
		} else if (matchTime > 55) {
			timeUntilHubActive = shift1Active ? 0 : matchTime - 55;
			timeUntilHubInactive = shift1Active ? matchTime - 55 : 0;
			return shift1Active;
		} else if (matchTime > 30) {
			timeUntilHubActive = !shift1Active ? 0 : matchTime - 30;
			timeUntilHubInactive = !shift1Active ? matchTime - 30 : 0;
			return !shift1Active;
		} else {
			timeUntilHubActive = 0;
			timeUntilHubInactive = 0;
			return true;
		}
	}


	@Override
	public void disabledPeriodic() {
	}


	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(autonomousCommand);
		}
	}


	/**
	 * This method is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}


	@Override
	public void teleopInit() {
		gameDataParsed = false;

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}


	/**
	 * This method is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	}


	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}


	/**
	 * This method is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}


	/**
	 * This method is called once when the robot is first started up.
	 */
	@Override
	public void simulationInit() {
	}


	/**
	 * This method is called periodically whilst in simulation.
	 */
	@Override
	public void simulationPeriodic() {
	}
}
