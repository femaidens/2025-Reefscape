// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveSim;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiL2 extends SequentialCommandGroup {
  /** Creates a new TaxiL2. */
  public TaxiL2(DriveSim drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    System.out.println("running!!!!");
    addCommands(
        new InstantCommand(() -> drive.zeroHeading()),
        new RunCommand(() -> drive.drive(() -> 0.05, () -> 0, () -> 0), drive).withTimeout(1),
        new InstantCommand(() -> drive.drive(() -> 0, () -> 0, () -> 0)),
        new RunCommand(() -> drive.drive(() -> 0.05, () -> 0, () -> 0))
    );
  }
}
