// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import monologue.Annotations.Log;
import monologue.Logged;

public class GeneralElevator extends SubsystemBase implements AutoCloseable, Logged{
  /** Creates a new GeneralElevator. */
  // private ElevatorIO elevator = (Robot.isReal()) ? (new ActualElevator()) :
  // (new Elevator());

  @Log.NT private double goal;

  private static DigitalInput botLimitSwitch = new DigitalInput(Ports.ElevatorPorts.BOT_SWITCH);

  public ElevatorIO elevator;

  public GeneralElevator(ElevatorIO hardware) {
    elevator = hardware;
  }

  public static GeneralElevator create() {
    return (Robot.isReal()) ? new GeneralElevator(new ActualElevator()) : new GeneralElevator(new Elevator());
  }

  public static GeneralElevator none() {
    return new GeneralElevator(new NoElevator());
  }

  @Log.NT
  private ProfiledPIDController elevatorPID = new ProfiledPIDController(
      Constants.ElevatorConstants.PIDConstants.kP,
      Constants.ElevatorConstants.PIDConstants.kI,
      Constants.ElevatorConstants.PIDConstants.kD,
      Constants.ElevatorConstants.PIDConstants.CONSTRAINTS);

  private ElevatorFeedforward ff = new ElevatorFeedforward(
      Constants.ElevatorConstants.FeedForwardConstants.kS,
      Constants.ElevatorConstants.FeedForwardConstants.kG,
      Constants.ElevatorConstants.FeedForwardConstants.kV);
  // private ElevatorIO elevator = new NoElevator();

  public void periodic() {
    // This method will be called once per scheduler run
    this.hitBotLimit();

  }

  @Override

  public void close() {
    this.close();
  }

  /**
   * if the limit switch is activated, the elevator motor stops moving
   */
  public void hitBotLimit() {
    if (botLimitSwitch.get()) {
      elevator.setVoltage(0);
    }
  }

  private void setGoal(double goal) {
    this.goal = goal;
    elevatorPID.setGoal(goal);// *Math.sin(0.61) */
  }

  private Command goal(double goal) {
    return run(() -> goal(goal));
  }

  private Command run() {
    return this.run(() -> {
      double pidOutput = elevatorPID.calculate(elevator.getPosition());
      double ffOutput = ff.calculate(elevatorPID.getSetpoint().velocity);
      elevator.setVoltage(pidOutput + ffOutput);
    });
  }

  public Command reachGoal(double goal) {
    return goal(goal).andThen(run());
  }

  public double position() {
    return elevator.getPosition();
  }
}