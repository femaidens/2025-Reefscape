// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*; 


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Ports.*;
import edu.wpi.first.wpilibj2.command.Command;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private int rainbowFirstPixelHue;
  private final Distance ledSpacing; 
  LEDPattern rainbow;


  public LED() {
     rainbow = LEDPattern.rainbow(255, 128);

    led = new AddressableLED(LEDPorts.LED_PORT);
    ledBuffer = new AddressableLEDBuffer(250);
    ledSpacing = Meters.of(1/5); 
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();

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
    LEDPattern scrollRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
    scrollRainbow.applyTo(ledBuffer);
    
    led.setData(ledBuffer); 
  }

  public void setGreenPurpleGradient(){
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kPurple); 
    gradient.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public void scrollGreenPurple(){
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,Color.kGreen, Color.kPurple); 
    LEDPattern pattern = gradient.scrollAtRelativeSpeed(Percent.per(Second).of(25)); 
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}
