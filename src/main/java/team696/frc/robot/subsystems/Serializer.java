// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team696.frc.lib.TalonFactory;
import team696.frc.robot.util.Constants;

public class Serializer extends SubsystemBase {
  private static Serializer m_Serializer;

  private TalonFactory _Serializer;

  private DigitalInput _FrontBeam;
  private DigitalInput _BackBeam;

  /** Creates a new Serializer. */
  private Serializer() {
    _Serializer = new TalonFactory(15, Constants.canivoreName, Constants.CONFIGS.shooter_Serializer, "Shooter Serializer");

    _FrontBeam = new DigitalInput(1);
    _BackBeam = new DigitalInput(0);
  }

  public static Serializer get() {
    if (m_Serializer == null) {
      m_Serializer = new Serializer();
    }

    return m_Serializer;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    _Serializer.PercentOutput(speed);
  }

  public void stop() {
    _Serializer.stop();
  }

  public Command feed() {
    return this.runEnd(()->_Serializer.PercentOutput(0.5), ()->_Serializer.stop());
  }

  public void serialize() {
    if (!_FrontBeam.get()) {
        _Serializer.stop();
      } else {
        if (!_BackBeam.get()) {
          _Serializer.PercentOutput(0.3);
        } else {
          _Serializer.PercentOutput(0.5);
        }
      }
  }

  public Command intake() {
    return this.runEnd(this::serialize, ()->_Serializer.stop());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Front Beam Break", ()->_FrontBeam.get(), null);
    builder.addBooleanProperty("Back Beam Break", ()->_BackBeam.get(), null);
  }
}
