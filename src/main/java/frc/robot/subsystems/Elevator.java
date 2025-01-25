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
import edu.wpi.first.wpilibj.Timer;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 NEO motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(4);

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
      1.0, //10
      Constants.kMaxElevatorHeightMeters,
      true,
      1, //10
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

  private final MechanismRoot2d m_mech2dRootStage1 = m_mech2d.getRoot("Elevator Root 1", 9,0);//stage 1
  private final MechanismRoot2d m_mech2dRootStage2 = m_mech2d.getRoot("Elevator Root 2", 10, 10); //stage 2
  private final MechanismRoot2d m_mech2dRootStage3 = m_mech2d.getRoot("Elevator Root 3", 11, 1); // stage 3

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
    m_elevatorSimStage3.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSimStage2.getPositionMeters());
    m_encoderSim.setDistance(m_elevatorSimStage3.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSimStage2.getCurrentDrawAmps()));
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSimStage3.getCurrentDrawAmps()));
  }

  public void setMotorVoltage(double voltage){
    m_motor.setVoltage(voltage);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public double reachGoal(double goal) {
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double distanceStage3 = m_elevatorSimStage3.getPositionMeters()*Constants.kStage3Velocity+m_elevatorMech2dStage1.getLength()-m_elevatorMech2dStage3.getLength(); //distance from bottom to stage 3 root
    double totalDistance = distanceStage3 + m_elevatorMech2dStage3.getLength(); //distance from the bottom to the top of stage 3
    double difference = goal - totalDistance;
    System.out.println("Goal difference: " + difference);
    if(difference < .25 && difference > -.25) {
      m_motor.setVoltage(0);
      System.out.println("Voltage when triggered: " + m_motor.getVoltage());
      System.out.println("at goal");
      return 0;
    }
    else{
      System.out.println("Not at goal");
      System.out.println("Length of stage 3: " + (m_elevatorMech2dStage1.getLength()+m_elevatorMech2dStage2.getLength() + m_elevatorMech2dStage3.getLength()));
      double pidOutput = m_controller.calculate(m_encoder.getDistance());
      double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
      m_motor.setVoltage(pidOutput + feedforwardOutput);
      System.out.println("Voltage when not triggered: " + m_motor.getVoltage());
      return pidOutput + feedforwardOutput;
    }
    
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
     m_elevatorMech2dStage2.setLength(7); //m_encoder.getDistance()
     m_elevatorMech2dStage3.setLength(5); //m_encoder.getDistance()
     m_mech2dRootStage1.setPosition(9.5, 0);
     m_mech2dRootStage2.setPosition(10,  m_elevatorMech2dStage1.getLength()-m_elevatorMech2dStage2.getLength() + m_encoder.getDistance()*Constants.kStage2Velocity); //m_elevatorSimStage3.getPositionMeters()
    m_mech2dRootStage3.setPosition(10.5, (m_elevatorMech2dStage1.getLength()-m_elevatorMech2dStage3.getLength())+ m_encoder.getDistance()*Constants.kStage3Velocity); //works m_elevatorSimStage2.getPositionMeters()+m_encoder.getDistance()
    m_elevatorMech2dStage1.setLength(10);
    double distanceStage2 = m_elevatorSimStage2.getPositionMeters()*Constants.kStage2Velocity+m_elevatorMech2dStage1.getLength()-m_elevatorMech2dStage2.getLength(); //distance from bottom to stage 2 root
    double distanceStage3 = m_elevatorSimStage3.getPositionMeters()*Constants.kStage3Velocity+m_elevatorMech2dStage1.getLength()-m_elevatorMech2dStage3.getLength(); //distance from bottom to stage 3 root
    double totalDistance = distanceStage3 + m_elevatorMech2dStage3.getLength(); //distance from the bottom to the top of stage 3
    if(distanceStage2 > m_elevatorMech2dStage1.getLength()) { //need to change this
      //m_motor.setVoltage(0);
      m_mech2dRootStage2.setPosition(10,  m_elevatorMech2dStage1.getLength());
      m_mech2dRootStage3.setPosition(10.5, (m_elevatorMech2dStage1.getLength()-m_elevatorMech2dStage3.getLength())+ m_encoder.getDistance()*Constants.kStage3Velocity);
      if (distanceStage3 > m_elevatorMech2dStage1.getLength()+m_elevatorMech2dStage2.getLength()) {
        //m_mech2dRootStage3.setPosition(10.5, m_elevatorMech2dStage2.getLength()+ m_elevatorMech2dStage1.getLength()-m_elevatorMech2dStage2.getLength() + m_encoder.getDistance()*Constants.kStage2Velocity);
        m_mech2dRootStage3.setPosition(10.5, m_elevatorMech2dStage1.getLength()+m_elevatorMech2dStage2.getLength());
      }
    }

    SmartDashboard.putNumber("Position Stage 2", distanceStage2);
    SmartDashboard.putNumber("GetPositionMeters Stage 2", m_elevatorSimStage2.getPositionMeters()); //distance from original position of stage 2
    SmartDashboard.putNumber("GetPositionMeters Stage 3", m_elevatorSimStage3.getPositionMeters()); //distance from original position of stage 3
    SmartDashboard.putNumber("Position Stage 3", distanceStage3);
    SmartDashboard.putNumber("Position Stage Total", totalDistance); 
    SmartDashboard.putNumber("kP", Constants.kElevatorKp); 
    SmartDashboard.putNumber("kI", Constants.kElevatorKi); 
    SmartDashboard.putNumber("kD", Constants.kElevatorKd); 
    SmartDashboard.putNumber("kS", Constants.kElevatorkS); 
    SmartDashboard.putNumber("kG", Constants.kElevatorkG); 
    SmartDashboard.putNumber("kV", Constants.kElevatorkV); 
    SmartDashboard.putNumber("kA", Constants.kElevatorkA); 
    SmartDashboard.putNumber("Velocity meters/s", m_elevatorSimStage2.getVelocityMetersPerSecond());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
}
