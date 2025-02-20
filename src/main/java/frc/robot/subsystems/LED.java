// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Ports.*;
import edu.wpi.first.wpilibj2.command.Command;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  int rainbowFirstPixelHue;

  public LED() {
    led = new AddressableLED(LEDPorts.LED_PORT);
    ledBuffer = new AddressableLEDBuffer(250);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }

  public void setDefault() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }

  public Command setDefaultCmd() {
    return this.run(() -> setDefault());
  }
  public Command setLEDBufferCmd() {
    return this.run(()-> led.setData(ledBuffer));
  }

  public void setLedPurple() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 160, 32, 340);
    }
    led.setData(ledBuffer);
  }

  public Command setPurpleCmd(){
    return this.run(() -> setLedPurple());
  }

  public void setLedGreen() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 225, 0);
    }
    led.setData(ledBuffer);
  }

  public Command setGreenCmd(){
    return this.run(() -> setLedGreen());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
