// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.commands;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.lib.PLog;
import team696.frc.robot.subsystems.Hood;
import team696.frc.robot.subsystems.Serializer;
import team696.frc.robot.subsystems.Shooter;
import team696.frc.robot.util.Constants;

public class ManualShot extends Command {

  boolean feed;
  Constants.shooter.state desiredState;

  double extraadded = 0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  DoubleTopic dblTopic = inst.getDoubleTopic("Manual Adjustment");
  DoubleSubscriber sub = dblTopic.subscribe(0);

  public ManualShot(Constants.shooter.state d) {
    desiredState = d;
    
    SmartDashboard.putNumber("Manual Adjustment", extraadded);

    addRequirements(Hood.get(), Serializer.get(), Shooter.get());
  }

  @Override
  public void initialize() {
    feed = false;
  }

  @Override
  public void execute() {
    extraadded = sub.get();
    Constants.shooter.state adesiredState = new Constants.shooter.state(desiredState.angle + extraadded, desiredState.speed_l, desiredState.speed_r);
    PLog.info("cock", String.format("%f",extraadded));
    Shooter.get().setShooter(adesiredState);
    Hood.get().setHood(adesiredState);

    if (Shooter.get().upToSpeed(adesiredState, 150) 
        && Hood.get().atAngle(adesiredState, 1.5)) {
          feed = true;
    }

    if (feed) {
      Serializer.get().setSpeed(1);
    } else {
      Serializer.get().stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Shooter.get().stop();
    Serializer.get().stop();
    Hood.get().stop();
  }

  @Override
  public boolean isFinished() {
    //if (Serializer.get().FrontBeam())
      //return true;

    return false;
  }
}
