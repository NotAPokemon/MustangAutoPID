package frc.team670.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team670.libs.health.HealthChecker;
import frc.team670.libs.logging.MustangNotifier;
import java.io.File;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command mAutonomousCommand;

  private final RobotContainer mRobotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    if (isReal()) {
      Logger.addDataReceiver(
          new WPILOGWriter("/U/logs")); // register log location for current run time Logs to a USB
      // stick
    } else {
      final String timestamp =
          DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")
              .format(LocalDateTime.now()); // Timestap of log
      // file
      final File logFolder = new File("logs"); // the directory for log file storage
      logFolder.mkdirs(); // attempt to create the director if it dosent exist already

      Logger.addDataReceiver(
          new WPILOGWriter("logs/" + timestamp + ".wpilog")); // register log location for current
      // run time logs to
      // logs/timestamp.wpilog within the
      // project directory
    }

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.start();

    mRobotContainer = new RobotContainer();
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
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    HealthChecker.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    try {
    } catch (Exception e) {
      MustangNotifier.error(e);
    }

    if (mAutonomousCommand != null) {
      mAutonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    mRobotContainer.periodic();
  }

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
