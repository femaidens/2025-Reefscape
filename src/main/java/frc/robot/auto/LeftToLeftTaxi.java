// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CoralTransition;
import frc.robot.commands.Elevating;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftToLeftTaxi extends SequentialCommandGroup {
  /** Creates a new BlueLeftToReefLeft. */
  public LeftToLeftTaxi(Drive drivetrain, Outtake outtake, Intake intake, Elevator elevator, Elevating elevating, CoralTransition coralTransition){
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroHeading()),
      new RunCommand(() -> drivetrain.drive(() -> 0.0, () -> 1.0, () -> 0.0), drivetrain)
        .withTimeout(5)
    );
  }
}
