// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds; 
import static edu.wpi.first.units.Units.Volts;

import java.util.Currency;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

//import com.ctre.phoenix6.hardware.TalonFX;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private static SparkMax elevatorMotorLeader;
  private static SparkMax elevatorMotorFollower;
  private static DigitalInput botLimitSwitch;
  private static PIDController elevatorPID;
  private static RelativeEncoder elevatorEncoder;
  private static AbsoluteEncoder absoluteEncoder;
  private static ElevatorFeedforward ff;
  private static double lastSetpoint;
  private final LED led; 
  // private double initialOffset = 0;

  private final SysIdRoutine.Config sysIDConfig = new SysIdRoutine.Config(Volts.of(2).per(Seconds),  // we don't know what seconds does but it works (if there's errors then it may be because of this)
  Volts.of(10),
  Seconds.of(10),
   null);

  private final SysIdRoutine elevatorRoutine = new SysIdRoutine(
    sysIDConfig, 
    new SysIdRoutine.Mechanism(
      volts -> setVoltage(volts.in(Volts)), null, this)); 
   
  public Elevator() {
    led = new LED(); 
    elevatorMotorLeader = new SparkMax(Ports.ElevatorPorts.LEADER_ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);
    elevatorMotorFollower = new SparkMax(Ports.ElevatorPorts.FOLLOWER_ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);
    
    botLimitSwitch = new DigitalInput(Ports.ElevatorPorts.BOT_SWITCH);
    
    elevatorEncoder = elevatorMotorLeader.getEncoder();
    absoluteEncoder = elevatorMotorLeader.getAbsoluteEncoder();

    elevatorPID = new PIDController(
      ElevatorConstants.PIDConstants.kP,
      ElevatorConstants.PIDConstants.kI,
      ElevatorConstants.PIDConstants.kD
    );

    elevatorPID.setTolerance(0.05);
  
    ff = new ElevatorFeedforward(
      ElevatorConstants.FeedForwardConstants.kS, 
      ElevatorConstants.FeedForwardConstants.kG, 
      ElevatorConstants.FeedForwardConstants.kV,
      ElevatorConstants.FeedForwardConstants.kA
    );

      SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);

        leaderConfig.encoder
        .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

      SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
        .follow(elevatorMotorLeader, true);

        followerConfig.encoder
        .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(Constants.ElevatorConstants.PIDConstants.kP, Constants.ElevatorConstants.PIDConstants.kI, Constants.ElevatorConstants.PIDConstants.kD);
        //probably unneeded 
        // config.absoluteEncoder
        // .zeroOffset(0.49);

        elevatorMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorEncoder.setPosition(absoluteEncoder.getPosition() - ElevatorConstants.ABSOLUTE_OFFSET);

        lastSetpoint = elevatorEncoder.getPosition();
    }
    
    public boolean atSetpoint(){
      // System.out.println(elevatorPID.atSetpoint());
      return elevatorPID.atSetpoint();
    }
  
    /**
     * sets motor voltage using calculations from PID and FF values 
     * 
     */
    public void elevatorPID(double current, double setpoint){
      // if (botLimitSwitch.get()) {
      //   elevatorMotorLeader.set(0);
      // }

      // else {
        elevatorMotorLeader.setVoltage(
          elevatorPID.calculate(current, setpoint));
          // elevatorMotorFollower.resumeFollowerMode();
      // }
       
    }
    // public void elevatorFF(double setpoint){
    //     elevatorMotorLeader.setVoltage(
    //       ff.calculate());
    // }


/**
 * see line 36, might not be needed. 
 * @return
//  */
    public void elevatorPIDw(double setpoint){
        double voltage = elevatorPID.calculate(elevatorEncoder.getPosition() ) + ff.calculate( elevatorPID.calculate(elevatorEncoder.getPosition()));
    // if(hitBotLimit()){
    //   elevatorMotorLeader.setVoltage(MathUtil.clamp(voltage, 0, 12));
    //     // not sure if this is correct
    //     // elevatorMotorFollower.resumeFollowerMode();
    // } else {
    //   elevatorMotorLeader.setVoltage(voltage);
    }
     

    /**
     * if the limit switch is activated, the elevator motor stops moving
     */
    // public void hitBotLimit(){
    //   if(botLimitSwitch.get()){
    //     stopMotorCmd();
    //   }
    // }

    public boolean hitBotLimit() {
      return !botLimitSwitch.get();
    }

    /**
     * see line 36
     */
    // public void botSwitchStatus(){
    //   if(previousSwitchTriggered && !currentSwitchTriggered){
    //     underBotSwitch = !underBotSwitch;
    //   }
    //   previousSwitchTriggered = currentSwitchTriggered;
    // }


    //Cmds
    /**
     * @return run elevator motor command
     */
    public Command runMotorCmd(){
      return this.run( () -> {
        elevatorMotorLeader.set(Constants.ElevatorConstants.MOTOR_SPEED);
        // System.out.println("FORWARD " + elevatorMotorLeader.getAppliedOutput());
      }
      );

    }

    /**
     * reverse motor
     */

     public Command reverseMotorCmd(){
      // if (botLimitSwitch.get()) {
      //   return this.run(() -> stopMotorCmd());
      // }
      // else {
        return this.run(() -> {
          // if(elevatorEncoder.getPosition() < ElevatorConstants.SetpointConstants.MINIMUM_LVL){
          if(hitBotLimit()){
            elevatorMotorLeader.stopMotor();
            //elevatorMotorFollower.stopMotor();
            elevatorEncoder.setPosition(0);
            System.out.println("ELEVATOR MOTOR STOPPED - BELOW LIMIT");
          } else {
            elevatorMotorLeader.set(-Constants.ElevatorConstants.REVERSE_MOTOR_SPEED);
            // System.out.println("reverseeeeeeee");
          }
        }
      );
    }

    public Command upToDefaultCmd() {
      return this.run(()-> {
        if(elevatorEncoder.getPosition() > ElevatorConstants.SetpointConstants.DEFAULT_LVL) {
          elevatorMotorLeader.stopMotor();
          System.out.println("REACHED DEFAULT LEVEL");
        }
        else {
          elevatorMotorLeader.set(-ElevatorConstants.MOTOR_SPEED);

        }
      });
    }

    public Command forceReverseMotorCmd(){
      return this.run(() ->
      elevatorMotorLeader.set(-ElevatorConstants.FORCE_MOTOR_SPEED));
    }

    public Command resetEncoder(){
      return this.runOnce(() ->
      elevatorEncoder.setPosition(ElevatorConstants.SetpointConstants.MINIMUM_LVL));
    }


    public Command stopMotorCmd(){
      return this.runOnce( () -> 
        elevatorMotorLeader.set(0)
      );
    }

    /**
     * 
     * @param setpoint
     * @return lifts elevator to specified setpoint
     */
    public Command setLevel(double setpoint) {
      return this.run(() -> {
        elevatorPID(elevatorEncoder.getPosition(), setpoint);
        // System.out.println(elevatorEncoder.getPosition());
        }
      ).alongWith(led.setProgressCmd());
    }

    /**
     * 
     * @param setpoint
     * @return lifts elevator to specified setpoint
     */
    public Command setLevelWithLimit(double setpoint) {
      return this.run(() -> {
        if(hitBotLimit()){
          elevatorMotorLeader.stopMotor();
          //elevatorMotorFollower.stopMotor();
          elevatorEncoder.setPosition(0);
          System.out.println("ELEVATOR MOTOR STOPPED - BELOW LIMIT");
        }  else {
            elevatorPID(elevatorEncoder.getPosition(), setpoint);
            // System.out.println(elevatorEncoder.getPosition());
        }}
      );
    }

    /**
     * Runs PID to stay at the last setpoint, which can be set using setCurrentSetpoint()
     * @return Run command
     */
    public Command stayAtLevel(){
        return setLevel(lastSetpoint);
    }


    public void setVoltage(double volts){
      elevatorMotorLeader.setVoltage(volts);
    }

    public double getCurrentPosition(){
      return elevatorEncoder.getPosition(); 
    }

    public Command setCurrentSetpoint(double setpoint){
        return this.runOnce(() -> lastSetpoint = setpoint);
    }

    public boolean atMinimum(){
      return getCurrentPosition() < ElevatorConstants.SetpointConstants.MINIMUM_LVL;
    }

    public boolean atMaximum(){
      return getCurrentPosition() > ElevatorConstants.SetpointConstants.MAXIMUM_LVL;
    }

    public Command quasiCmd(SysIdRoutine.Direction direction) {
      return elevatorRoutine.quasistatic(direction);
    }

    public Command dynaCmd(SysIdRoutine.Direction direction) {
      return elevatorRoutine.dynamic(direction);
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // this.hitBotLimit();
   SmartDashboard.putData(this);
   SmartDashboard.putBoolean("Bottom Limit Switch", hitBotLimit());
   SmartDashboard.putNumber("encoder", elevatorEncoder.getPosition());
   SmartDashboard.putNumber("absolute encoder", absoluteEncoder.getPosition());
   SmartDashboard.putNumber("current position", getCurrentPosition());
  //  SmartDashboard.putBoolean("Under limit switch", underBotSwitch);
  //  botSwitchStatus();
   SmartDashboard.updateValues();
  }
}
// }
