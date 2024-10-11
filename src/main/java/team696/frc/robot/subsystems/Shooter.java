// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.lib.HardwareDevices.TalonFactory;
import team696.frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private static Shooter m_Shooter;

  private TalonFactory _LeftShooter;
  private TalonFactory _RightShooter;

  private VelocityVoltage _VelocityControllerL;
  private VelocityVoltage _VelocityControllerR;

  /** Creates a new Shooter. */
  private Shooter() {
    _LeftShooter = new TalonFactory(13, Constants.canivoreName, Constants.configs.shooter.left, "Shooter Left Shooter");
    _RightShooter = new TalonFactory(14, Constants.canivoreName, Constants.configs.shooter.right, "Shooter Right Shooter");

    _VelocityControllerL = new VelocityVoltage(0);
    _VelocityControllerR = new VelocityVoltage(0);

    this.setDefaultCommand(this.SmartIdleShooter());
  }

  public static Shooter get() {
    if (m_Shooter == null) {
      m_Shooter = new Shooter();
    }

    return m_Shooter;
  }

  public double getLeftVelocity() {
    return _LeftShooter.getVelocity() * 60;
  }

  public double getRightVelocity() {
    return _RightShooter.getVelocity() * 60;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean upToSpeed(double speed_L, double speed_R, double tolerance) {
      if (getLeftVelocity() < speed_L - tolerance || getLeftVelocity() > speed_L + tolerance) return false;
      if (getRightVelocity() < speed_R - tolerance || getRightVelocity() > speed_R + tolerance) return false;

      return true;
  }

  public boolean upToSpeed(Constants.shooter.state desired, double tolerance) {
    return upToSpeed(desired.speed_l, desired.speed_r, tolerance);
  }

  public void setShooter(Constants.shooter.state desired) {
    setShooter(desired.speed_l, desired.speed_r);
  }

  public void setShooter(double l, double r) {
    _LeftShooter.setControl(_VelocityControllerL.withVelocity(l/60.0));
    _RightShooter.setControl(_VelocityControllerR.withVelocity(r/60.0));
  }

  public void stop() {
    _LeftShooter.stop();
    _RightShooter.stop();
  }

  public void setShooterPercent(double l, double r) {
    _LeftShooter.PercentOutput(l);
    _RightShooter.PercentOutput(r);
  }

  public Command spinShooter(double speed) {
    return this.runEnd(()-> setShooterPercent(speed, speed), this::stop);
  }

  public Command SmartIdleShooter() {
    return this.runEnd(()-> 
    {
      if (DriverStation.isAutonomous() && Swerve.get().getDistToSpeaker() < 4.5) {
        setShooterPercent(0.50, 0.50);
      } else {
        setShooterPercent(0.05, 0.05);
      }
    } ,this::stop);
  }

  public Command spinShooter(double speedl, double speedr) {
    return this.runEnd(()-> setShooterPercent(speedl, speedr), this::stop);
  }

  public Command spinShooterRPM(double speed) {
    return this.runEnd(()->setShooter(speed, speed), this::stop);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
    builder.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
  }
}