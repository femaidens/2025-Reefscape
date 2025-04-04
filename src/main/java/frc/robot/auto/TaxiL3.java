// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiL3 extends SequentialCommandGroup {
  /** Creates a new TaxiL3. */
  public TaxiL3(Elevating elevating, Outtake outtake, Vision vision, CoralTransition coralTransition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> vision.visionZeroHeading())
        // new RunCommand(() -> vision.funkierRight()).alongWith(coralTransition.moveCoralToOuttake()).withTimeout(4),
        // elevating.thirdLevelCmd().until(elevating.elevator::atSetpoint),
        // outtake.runMotorCmd().withTimeout(3)
    );
  }
}
