package team696.frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team696.frc.robot.commands.TeleopSwerve;
import team696.frc.robot.subsystems.Swerve;
import team696.frc.robot.util.Auto;
import team696.frc.robot.util.PVCamera;
import team696.frc.robot.util.Constants;
import team696.frc.robot.util.Controls;
import team696.frc.robot.util.Util;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private PowerDistribution m_PDH;

    @Override
    public void robotInit() {
        Util.setRobotType();

        Logger.recordMetadata("ProjectName", "2024OffSeason"); // Set a metadata value

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
          case 0:
            Logger.recordMetadata("GitDirty", "All changes committed");
            break;
          case 1:
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
          default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
        }

        switch (Constants.Robot.detected) {
          case SIM:
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            break;
          case COMP:
          case BETA:
          case UNKNOWN:
          default:
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            break;
        }
        m_PDH = new PowerDistribution(1, ModuleType.kRev); 
        
        Logger.start(); 

        DriverStation.silenceJoystickConnectionWarning(true);
    
        LiveWindow.disableAllTelemetry();
        RobotController.setEnabled3V3(false);
        RobotController.setEnabled5V(false);
        RobotController.setEnabled6V(false);

        m_PDH.setSwitchableChannel(true);
        
        SmartDashboard.putData(Swerve.get());

        configureControllerBinds();

        Auto.Initialize();

        PVCamera.get();
    }

  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = Auto.Selected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override 
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() { }

  @Override
  public void simulationInit() { }

  @Override
  public void simulationPeriodic() { 
    PVCamera.get().simPeriodic(Swerve.get().getPose());
  }

  @SuppressWarnings("unused") 
    private void configureBinds() {
      TeleopSwerve.config(Controls.leftJoyX, Controls.leftJoyY, Controls.rightJoyX, null, Constants.deadBand);
      Swerve.get().setDefaultCommand(new TeleopSwerve());
      Controls.leftJoy.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
    }

    @SuppressWarnings("unused") 
    private void configureOperatorBinds() {
    
    }

    @SuppressWarnings("unused") 
    private void configureControllerBinds() { 
      TeleopSwerve.config(Controls.Controller.leftJoyX, Controls.Controller.leftJoyY, Controls.Controller.rightJoyX, null, 0.02);
      Swerve.get().setDefaultCommand(new TeleopSwerve());
      Controls.Controller.A.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
    }
}
