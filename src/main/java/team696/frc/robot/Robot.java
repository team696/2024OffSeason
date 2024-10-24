package team696.frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team696.frc.lib.Auto;
import team696.frc.lib.Util;
import team696.frc.lib.Auto.NamedCommand;
import team696.frc.lib.Dashboards.ShuffleDashboard;
import team696.frc.lib.Dashboards.WebDashboard;
import team696.frc.robot.commands.Amp;
import team696.frc.robot.commands.AutoDriveTowardsNote;
import team696.frc.robot.commands.Drop;
import team696.frc.robot.commands.Shoot;
import team696.frc.robot.commands.ShooterIntake;
import team696.frc.robot.commands.TeleopSwerve;
import team696.frc.robot.commands.GroundIntake;
import team696.frc.robot.commands.HoldPosition;
import team696.frc.robot.commands.ManualShot;
import team696.frc.robot.commands.Pass;
import team696.frc.robot.commands.Rotate;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Intake;
import team696.frc.robot.subsystems.LED;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;
import team696.frc.robot.subsystems.Swerve;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private PowerDistribution m_PDH;

    @Override
    public void robotInit() {
        ShuffleDashboard.initialize();

        Logger.recordMetadata("ProjectName", "2024OffSeason");
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
        RobotController.setEnabled5V(true);
        RobotController.setEnabled6V(false);

        m_PDH.setSwitchableChannel(true);
        
        SmartDashboard.putData(Swerve.get());
        SmartDashboard.putData(Shooter.get());
        SmartDashboard.putData(Intake.get());
        SmartDashboard.putData(Hood.get());
        SmartDashboard.putData(Serializer.get());
        SmartDashboard.putData(LED.get());

        Auto.Initialize(Swerve.get(), 
          new NamedCommand("ShootIntakeShoot", ((((new Shoot()).andThen(new GroundIntake().raceWith(Shooter.get().SmartIdleShooter()))).andThen(new Shoot())).asProxy())),
          new NamedCommand("IntakeShoot", (((new GroundIntake()).andThen(new Shoot())).asProxy())),
          new NamedCommand("ShootFree", ((new Shoot()).asProxy())),
          new NamedCommand("Shoot", (new Shoot()).asProxy().deadlineWith(new Rotate())),
          new NamedCommand("Intake", (new GroundIntake().raceWith(Shooter.get().SmartIdleShooter())).asProxy()),
          new NamedCommand("ShootIntakeShootFree", ((((new Shoot()).andThen(new GroundIntake(false).raceWith(Shooter.get().SmartIdleShooter()))).andThen(new Shoot())).asProxy())),
          new NamedCommand("Drop", (new ManualShot(new Constants.shooter.state(0, 2500, 2500))).asProxy()),
          new NamedCommand("Subwoofer", (new ManualShot(new Constants.shooter.state(4.7, 3800, 3900)))),
          new NamedCommand("NotePickup", new AutoDriveTowardsNote(false)),
          new NamedCommand("Rotate", new Rotate())
        );

        configureBinds();
        configureOperatorBinds();
        
        //configureControllerBinds();
        //configureWebControlsBinds();
      }

  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() { }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = Auto.SelectedEndAt15();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override 
  public void autonomousExit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {

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

  @SuppressWarnings("unused") 
    private void configureBinds() {
      TeleopSwerve.config(Controls.leftJoyX, Controls.leftJoyY, Controls.rightJoyX, Controls.rightJoyB, Constants.deadBand);
      Swerve.get().setDefaultCommand(new TeleopSwerve(()->Swerve.get().getAngleToSpeaker()));
      Controls.leftJoyB.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
      Controls.Left.onTrue(Commands.run(()->Swerve.get().updateYawOffset()));
    }

    @SuppressWarnings("unused") 
    private void configureOperatorBinds() {
      Controls.Amp.whileTrue(new Amp(Controls.Rollers)
       .alongWith(new TeleopSwerve(
        () ->  0.75,
        ()-> Util.getAlliance() == Alliance.Red ? Constants.Field.RED.Amp.getRotation() : Constants.Field.BLUE.Amp.getRotation()
        )).alongWith(LED.get().LerpColor(()->Swerve.get().distTo(Util.getAlliance() == Alliance.Red ? Constants.Field.RED.Amp.getTranslation()           : Constants.Field.BLUE.Amp.getTranslation())*4)));
      Controls.Shoot.whileTrue(new Shoot());
      Controls.Drop.whileTrue(new Drop());

      Controls.Ground.whileTrue((new GroundIntake()).alongWith(new TeleopSwerve(Swerve.get()::getAngleForNote)));
      Controls.Source.whileTrue((new ShooterIntake()).alongWith(new TeleopSwerve(()->Util.getAlliance() == Alliance.Red ? Constants.Field.RED.Source.getRotation() : Constants.Field.BLUE.Source.getRotation())));

      Controls.ExtraA.whileTrue(new ManualShot(new Constants.shooter.state(4.7, 3800, 3900)));

      Controls.Trap.whileTrue(new Pass().alongWith(new TeleopSwerve(()->Swerve.get().getAngleToCorner())));

      Controls.Rightest.whileTrue(Auto.PathFind(Constants.Field.BLUE.Amp));

      Controls.Right.whileTrue((new HoldPosition(Constants.Field.BLUE.Amp)).alongWith(LED.get().LerpColor(()->Swerve.get().getState().velocity()*3)));

      //Controls.Right.whileTrue(Auto.get().PathFindToAutoBeginning());
    }

    @SuppressWarnings("unused") 
    private void configureControllerBinds() { 
      TeleopSwerve.config(Controls.XBOXController.leftJoyX, Controls.XBOXController.leftJoyY, Controls.XBOXController.rightJoyX, Controls.XBOXController.RB, 0.02);
      Swerve.get().setDefaultCommand(new TeleopSwerve(()->Swerve.get().getAngleToSpeaker()));
      Controls.XBOXController.A.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));

      Controls.XBOXController.RT.whileTrue(new Shoot());

      Controls.XBOXController.Y.whileTrue(new ShooterIntake());

      Controls.XBOXController.X.whileTrue(new Drop());

      Controls.XBOXController.B.whileTrue(new GroundIntake());

      Controls.XBOXController.LB.whileTrue(new Amp(Controls.XBOXController.LT).alongWith(new TeleopSwerve(()-> Util.getAlliance() == Alliance.Red ? Constants.Field.RED.Amp.getRotation() : Constants.Field.BLUE.Amp.getRotation())));
    }

    @SuppressWarnings("unused")
    private void configureWebControlsBinds() {
      TeleopSwerve.config(()->WebDashboard.getData("inputX").getDouble(), ()->WebDashboard.getData("inputY").getDouble(), ()->0, null, 0.05);
      Swerve.get().setDefaultCommand(new TeleopSwerve(()->Swerve.get().getAngleToSpeaker()));
    }
}
