package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import frc.robot.util.Controls;
import frc.robot.util.Util;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private PowerDistribution m_PDH;
  
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
      TeleopSwerve.config(Controls.Controller.leftJoyX, Controls.Controller.leftJoyY, Controls.Controller.rightJoyX, null, 0.08);
      Swerve.get().setDefaultCommand(new TeleopSwerve());
      Controls.Controller.A.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
    }

    @Override
    public void robotInit() {
        Util.setRobotType();
        
        m_PDH = new PowerDistribution(1, ModuleType.kRev);

        DriverStation.silenceJoystickConnectionWarning(true);
    
        LiveWindow.disableAllTelemetry();
        RobotController.setEnabled3V3(false);
        RobotController.setEnabled5V(false);
        RobotController.setEnabled6V(false);

        m_PDH.setSwitchableChannel(true);
        
        SmartDashboard.putData(Swerve.get());

        configureControllerBinds();
    }

  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      if (Constants.DEBUG) 
          SmartDashboard.putData(m_PDH);
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
    m_autonomousCommand = null;

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
  public void simulationPeriodic() { }
}
