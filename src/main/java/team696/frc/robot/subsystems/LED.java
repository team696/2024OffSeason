// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team696.frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.SingleFadeAnimation;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static LED _LED;

  private CANdle _candle;

  private final int _ledOffset = 8;
  private final int _numLed = 36 + 8;

  public static LED get() {
    if (_LED == null) {
      _LED = new LED();
    }

    return _LED;
  }

  private LED() {
    _candle = new CANdle(0);
    _candle.configFactoryDefault();

    CANdleConfiguration _candleConfiguration = new CANdleConfiguration(); //Only here because phoenix v5 shitty

    _candleConfiguration.statusLedOffWhenActive = true;
    _candleConfiguration.disableWhenLOS = false;
    _candleConfiguration.stripType = LEDStripType.RGB;
    _candleConfiguration.brightnessScalar = 1;
    _candleConfiguration.vBatOutputMode = VBatOutputMode.On;
    _candleConfiguration.enableOptimizations = true;
    _candleConfiguration.v5Enabled = true;

    _candle.configAllSettings(_candleConfiguration);

    for(int i = 0; i<_candle.getMaxSimultaneousAnimationCount(); i++) {
      _candle.clearAnimation(i);
    }

    this.setDefaultCommand(HasNote()); // LarsonAnimation( 255,  50, 60, 255, 0.3, _numLed, BounceMode.Front, 6, _ledOffset )
  }

  @Override
  public void periodic() {

  }

  public void setColor(int r, int g, int b) {
    _candle.clearAnimation(0);

    _candle.setLEDs(r,g,b, 255, _ledOffset, _numLed);
  }

  public void setAnimation(Animation anim) {
    _candle.animate(anim,0);
  }

  public Command Color(int r, int g, int b) {
    return (this.startEnd(()->setColor(r,g,b),()->{})).ignoringDisable(true);
  }

  public Command LerpColor(DoubleSupplier t) {
    return this.run(()->{setColor((int)(t.getAsDouble() * 255), (int)(255-t.getAsDouble() * 255), 0);});
  }

  public Command Animate(Animation anim) {
    return (this.startEnd(()->setAnimation(anim),()->{})).ignoringDisable(true);
  }
  
  public Command HasNote(){
    return this.run(()->{
      if(!Serializer.get().BackBeam()){
        setAnimation(new SingleFadeAnimation(0, 255, 0));
      }else{
        if(RobotController.getRSLState()) { setColor(255,20,0); } else { setColor(0,0,0);}
      }
    }).ignoringDisable(true);
  }

  public Command MatchRSL() {
    return (this.run(()-> {if(RobotController.getRSLState()) { setColor(255,20,0); } else { setColor(0,0,0);}})).ignoringDisable(true);
  }
}
