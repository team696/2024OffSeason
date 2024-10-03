// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.lib.Util;
import team696.frc.lib.Swerve.SwerveConstants;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Swerve;

public class AutoDriveTowardsNote extends Command {

    private static PIDController rotationController = new PIDController(0.015, 0, 0);

    static {
      rotationController.setSetpoint(0);
    }

    private boolean goingSideways = false;

  /** Creates a new AutoDriveTowardsNote. */
  public AutoDriveTowardsNote(boolean goingSideways) {
    this.goingSideways = goingSideways;

    addRequirements(Swerve.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      Swerve.get().Drive(new Translation2d(0.4 * SwerveConstants.maxSpeed, 0), 
                rotationController.calculate(Swerve.get().intakeCam.tX()), 
                false, true); 
  
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Swerve.get().doNothing();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Serializer.get().BackBeam()) return true; 
  
    if (!goingSideways) {
      if (Util.getAlliance() == Alliance.Blue &&  Swerve.get().getPose().getX() > 8.3 - 0.1) return true;

      if (Util.getAlliance() == Alliance.Red  &&  Swerve.get().getPose().getX() < 8.3 + 0.1) return true;
    }

    return false;
  }
}
