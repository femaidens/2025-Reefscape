// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Elevator implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.kElevatorKp,
          Constants.kElevatorKi,
          Constants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));

  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);

  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel); // its a relative encoder

  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort); 

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSimStage1 = 
  new ElevatorSim(
    m_elevatorGearbox,
      Constants.kElevatorGearing,
      Constants.kCarriageMass,
      Constants.kElevatorDrumRadius,
      10.0,
      Constants.kMaxElevatorHeightMeters,
      true,
      10,
      0.01,
      0.0);
  private final ElevatorSim m_elevatorSimStage2 =
      new ElevatorSim(
          m_elevatorGearbox,
          Constants.kElevatorGearing,
          Constants.kCarriageMass,
          Constants.kElevatorDrumRadius,
          Constants.kMinElevatorHeightMeters,
          Constants.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);
  private final ElevatorSim m_elevatorSimStage3 =
          new ElevatorSim(
              m_elevatorGearbox,
              Constants.kElevatorGearing,
              Constants.kCarriageMass,
              Constants.kElevatorDrumRadius,
              Constants.kMinElevatorHeightMeters,
              Constants.kMaxElevatorHeightMeters,
              true,
              0,
              0.01,
              0.0);
      
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);

  private final MechanismRoot2d m_mech2dRootStage1 = m_mech2d.getRoot("Elevator Root 1", 9.75,0);//stage 1
  private final MechanismRoot2d m_mech2dRootStage2 = m_mech2d.getRoot("Elevator Root 2", 10, 10); //stage 2
  private final MechanismRoot2d m_mech2dRootStage3 = m_mech2d.getRoot("Elevator Root 3", 10.25, 1); // stage 3

  private final Color8Bit stage1 = new Color8Bit(Color.kRed);
  private final Color8Bit stage2 = new Color8Bit(Color.kGreen);
  private final Color8Bit stage3 = new Color8Bit(Color.kBlue);


  private MechanismLigament2d m_elevatorMech2dStage1 =
      m_mech2dRootStage1.append(
        new MechanismLigament2d("Elevator1", m_elevatorSimStage1.getPositionMeters(), 90, 5, stage1)
      );
  private MechanismLigament2d m_elevatorMech2dStage2 = 
      m_mech2dRootStage2.append(
          new MechanismLigament2d("Elevator2", m_elevatorSimStage2.getPositionMeters(), 90, 5, stage2));
  private MechanismLigament2d m_elevatorMech2dStage3 = 
      m_mech2dRootStage3.append(
        new MechanismLigament2d("Elevator3", m_elevatorSimStage3.getPositionMeters(), 90, 5, stage3)
      );

  /** Subsystem constructor. */
  public Elevator() {
    m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSimStage2.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
    m_elevatorSimStage1.setInput(0);
    m_elevatorSimStage3.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSimStage2.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSimStage2.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSimStage2.getCurrentDrawAmps()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal) {
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }

  public void setStageTwoStart(){
    // m_elevatorSim2.setState(m_elevatorSim1.getPositionMeters(), m_elevatorSim1.getVelocityMetersPerSecond());
    m_mech2dRootStage2.setPosition(10.25,  m_elevatorMech2dStage1.getLength()); //m_elevatorSimStage3.getPositionMeters()
    m_mech2dRootStage3.setPosition(10.5, m_elevatorMech2dStage1.getLength()+m_elevatorMech2dStage2.getLength()); //works m_elevatorSimStage2.getPositionMeters()+m_encoder.getDistance()
    m_mech2dRootStage1.setPosition(10, 0);
    // System.out.println("updating"); 
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2dStage2.setLength(m_encoder.getDistance());
    m_elevatorMech2dStage3.setLength(m_encoder.getDistance());
    m_elevatorMech2dStage1.setLength(10);
    SmartDashboard.putNumber("Position", m_elevatorSimStage2.getPositionMeters());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
}
