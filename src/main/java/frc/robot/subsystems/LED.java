// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Ports.*;
import edu.wpi.first.wpilibj2.command.Command;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private int rainbowFirstPixelHue;
  private final Distance ledSpacing; 
  // private final Elevator elevator;
  LEDPattern rainbow;

  // Colors!!

  Color gold, pink, green, purple, orange, turquoise, brown,red; 


  public LED() {
    // elevator = new Elevator();
     rainbow = LEDPattern.rainbow(255, 100);

    led = new AddressableLED(LEDPorts.LED_PORT);
    ledBuffer = new AddressableLEDBuffer(250);
    ledSpacing = Meters.of(1/75); 
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();


    //COLORS ARE GRB
  gold = new Color(215, 255, 0); 
  pink = new Color(0,255,127);
  green = new Color(255,0,0);
  purple = new Color(32,115,240); 
  orange = new Color(156, 230,28); 
  turquoise = new Color(255, 64,208);
  brown = new Color(32,61,9);
  red = new Color(0,255,0); 
  }

  public Command setDefaultCmd(Command command) {
    return this.run(() -> setDefault());
  }

  public Command setLEDBufferCmd() {
    return this.run(()-> led.setData(ledBuffer));
  }

  public Command setPurpleCmd(){
    return this.run(() -> setLedPurple());
  }

  public Command setGreenCmd(){
    return this.run(() -> setLedGreen());
  }

  public Command setRainbowScrollCmd(){
    return this.run( () -> setRainbowScroll());
  }

  public Command setRainbowCmd(){
    return this.run( () -> setRainbow()); 
  }

  public Command setContGoldPinkCmd(){
    return this.run(() -> continuousGoldPink());
  }

  public Command setGradGPCmd(){
    return this.run(() -> setGreenPurpleGradient()); 
  }

  public Command setScrollCrazyRainbowCmd(){
    return this.run(() -> scrollCrazyRainbow()); 
  }

  public Command setScrollGPCmd(){
    return this.run(() -> scrollGP()); 
  }

  public Command setColorCmd(){
    return this.run(() -> setColor(brown)); 
  }

  public Command setRedBlinkCmd(){
    return this.run(() -> setRed());
  }

  public Command setPinkCmd(){
    return this.run(() -> setPink()); 
  }

  public Command setTurquoiseCmd(){
    return this.run(() -> setTurquoise()); 
  }

  public Command setBlinkCmd(){
    return this.run(() -> setBlink(orange));
  }

  // public Command setProgressCmd(){
  //   return this.run(() -> setProgress());
  // }

  public Command setBreatheCmd(){
    return this.run(()-> setBreathe());
  }

  public Command setOrangeCmd(){
    return this.run(() -> setColor(orange));
  }

  public void setLedGreen() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 2, 48, 32);
    }
    led.setData(ledBuffer);
  }

  public void setLedPurple() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 48, 25, 52);
    }
    led.setData(ledBuffer);
  }

  public void setDefault() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }

  public void setRainbow(){
    rainbow.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void setRainbowScroll(){
    LEDPattern rainbows = LEDPattern.rainbow(255, 100);
    LEDPattern scrollRainbow = rainbows.scrollAtRelativeSpeed(Percent.per(Second).of(50));
    scrollRainbow.applyTo(ledBuffer);
    
    led.setData(ledBuffer); 
  }
  



  public void setGreenPurpleGradient(){
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, green, purple); 
    gradient.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void scrollCrazyRainbow(){
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,Color.kDarkGreen,orange,gold,green,Color.kOrchid,Color.kAliceBlue,pink); 
    LEDPattern pattern = gradient.scrollAtRelativeSpeed(Percent.per(Second).of(50)); 
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
}

public void scrollGP(){
  LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, green, Color.kMediumTurquoise); 
  LEDPattern pattern = gradient.scrollAtRelativeSpeed(Percent.per(Second).of(50)); 
  pattern.applyTo(ledBuffer);
  led.setData(ledBuffer);
}

 public void continuousGoldPink(){
  LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, gold,pink);
  LEDPattern pattern = gradient.scrollAtRelativeSpeed(Percent.per(Second).of(50));
  pattern.applyTo(ledBuffer);
  led.setData(ledBuffer);
 }

 public void setPink(){
  LEDPattern p = LEDPattern.solid(pink); 
  p.applyTo(ledBuffer); 
  led.setData(ledBuffer);
 }

 public void setTurquoise(){
  LEDPattern p = LEDPattern.solid(turquoise); 
  p.applyTo(ledBuffer); 
  led.setData(ledBuffer);
 }

 public void setRed(){
  LEDPattern p = LEDPattern.solid(red);
  p.applyTo(ledBuffer);
  led.setData(ledBuffer);
 }

public void setColor(Color x){
  LEDPattern p = LEDPattern.solid(x); 
  p.applyTo(ledBuffer);
  led.setData(ledBuffer);
}

public void setBlink(Color x){
  LEDPattern base = LEDPattern.solid(x);
  LEDPattern pattern = base.blink(Seconds.of(0.8));
  pattern.applyTo(ledBuffer);
  led.setData(ledBuffer);
}

// public void setProgress(){
//   LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,pink,purple);
//   LEDPattern mask = LEDPattern.progressMaskLayer(() -> elevator.getCurrentPosition()/Constants.ElevatorConstants.SetpointConstants.MAXIMUM_LVL); 
//   LEDPattern heightDisplay = base.mask(mask); 
//   heightDisplay.applyTo(ledBuffer);
//   led.setData(ledBuffer);

// }



public double getH(){
  return 0.5;
}

public double getMaxH(){
  return 1; 
}

public void setBreathe(){
  LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, green);
  LEDPattern pattern = base.breathe(Seconds.of(2));
  pattern.applyTo(ledBuffer);
  led.setData(ledBuffer);
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}