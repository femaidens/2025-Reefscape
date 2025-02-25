// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class CoralTransition {
  Intake intake;
  Outtake outtake;

  public CoralTransition(Intake intake, Outtake outtake) {
    this.intake = intake;
    this.outtake = outtake;

  }

  /*
   * @ngozi, emily, yujing
   * i think this needs editing, need to look at both intake beam break and
   * outtake beam break. IF the intake BB off while outtake BB on, it'll stop.
   */
  public boolean hasCoral() {
    return !intake.isBeamBroken() && outtake.isBeamBroken();
  }

  public Command moveCoralToOuttake() {
    return intake.runMotorCmd()
        .alongWith(outtake.setOuttakeCoralSpeedCmd())
        .until(this::hasCoral)
        .andThen(intake.stopMotorCmd())
        .alongWith(outtake.stopMotorCmd());

  }

}