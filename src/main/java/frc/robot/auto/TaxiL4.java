// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Vision;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiL4 extends SequentialCommandGroup {
  /** Creates a new TaxiL4. */
  public TaxiL4(Elevating elevating, Outtake outtake, Vision vision, CoralTransition coralTransition, Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> vision.visionZeroHeading()),
         
        vision.funkierRight().withTimeout(3), // .alongWith(coralTransition.moveCoralToOuttake()).withTimeout(4)
        drive.setStraightCmd().withTimeout(0.5),
        elevating.fourthLevelCmd().alongWith(vision.stopDriving()).withTimeout(2.7),
        elevating.fourthLevelCmd().alongWith(outtake.runMotorCmd()).withTimeout(.65),
        elevating.scoringAlgaeBargeCmd().withTimeout(.7),
        outtake.stopMotorCmd(),
        elevating.resetDefault().withTimeout(2),
        new RunCommand(() -> vision.driveFromVision(() -> 0,() -> 0,() -> 0.2), vision).withTimeout(2.65)
        .andThen(new InstantCommand(() -> vision.visionZeroHeading()))
        // new RunCommand(() -> vision.drive.drive(() -> 0.0, () -> 0.0, () -> 0.2))// vision.driveFromVision(() -> 0.0, ()   // -> 0.0, () -> 0.2))
        // .withTimeout(3),
        // new InstantCommand(() -> vision.visionZeroHeading()));
        );
  }
}
