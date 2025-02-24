// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.Elastic;
import lib.Elastic.ElasticNotification;
import lib.Elastic.ElasticNotification.NotificationLevel;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.reduxrobotics.canand.CanandEventLoop;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final PowerDistribution m_powerDistribution;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Instantiate the power distribution
    // Clear the sticky faults
    m_powerDistribution = new PowerDistribution();
    m_powerDistribution.clearStickyFaults();
  }

  @Override
  public void robotInit() {
    // * DISABLE LIVE WINDOW
    LiveWindow.disableAllTelemetry();

    // * DISABLE PHOENIX LOGGING
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    // * Cameras port forwarding over USB
    // for (int port = 5800; port <= 5807; port++) PortForwarder.add(port, Constants.Vision.Limelight3.kName + ".local", port);
    for (int port = 5800; port <= 5807; port++) PortForwarder.add(port, Constants.Vision.Limelight3G.kName + ".local", port);
    // for (int port = 5800; port <= 5807; port++) PortForwarder.add(port, Constants.Vision.LimelightTwoPlus.kName + ".local", port);
    for (int port = 5800; port <= 5807; port++) PortForwarder.add(port, "photonvision.local", port);

    // * DataLogManager
    try {
      DataLogManager.start();
      DataLogManager.logNetworkTables(true);
      DriverStation.startDataLog(DataLogManager.getLog(), true);
      Elastic.sendAlert(new ElasticNotification(NotificationLevel.INFO, "DataLogManager started", "DataLogManager started successfully."));
    } catch (Exception e) {
      Elastic.sendAlert(new ElasticNotification(NotificationLevel.ERROR, "DataLogManager failed to start", "DataLogManager failed to start."));
    }

    // Start Reduxlib server
    CanandEventLoop.getInstance();
    
    //! ADVANTAGE KIT LOGGING
    Logger.addDataReceiver(new NT4Publisher());
    if (Constants.Logging.kUseURCL) Logger.registerURCL(URCL.startExternal());
    Logger.start();
    URCL.start();

    // * Initialization alert
    Elastic.sendAlert(new ElasticNotification(NotificationLevel.INFO, "Robot ready!", "Wait for subsystem initialization to complete."));

    // * Path finding warmup
    System.out.println("Pathfinding warmup...");
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    // Update CommandScheduler on the dashboard
    if (Constants.Logging.kDebug) SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Elastic.sendAlert(new ElasticNotification(NotificationLevel.INFO, "Robot Disabled.", "Robot has been disabled."));
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Elastic.sendAlert(new ElasticNotification(NotificationLevel.WARNING, "Robot Enabled Autonomous.", "Robot has been enabled in autonomous mode, BE CAUTIOUS."));

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Elastic.sendAlert(new ElasticNotification(NotificationLevel.WARNING, "Robot Enabled Teleop.", "Robot has been enabled in Teleop mode, BE CAUTIOUS."));
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
