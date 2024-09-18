// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Swerve;
import team696.frc.robot.util.LLCamera;
/**
 * failing rn
 */
public class driveToTarget extends Command {

  PIDController thetaController; 
   /**
   * lmao
   */
  public driveToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    thetaController=new PIDController(.1, 0, 0);
    addRequirements(Swerve.get());
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LLCamera.get().seesNote())
    Swerve.get().Drive(new Translation2d(0.3, 0),thetaController.calculate(Swerve.get().getYaw().getDegrees(), Swerve.get().getYaw().getDegrees()-LLCamera.get().getAngleForNote()), false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Swerve.get().Drive(new Translation2d(0,0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Serializer.get().BackBeam()&&!LLCamera.get().seesNote();
  }
}
