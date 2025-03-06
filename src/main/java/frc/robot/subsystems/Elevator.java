// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds; 
import static edu.wpi.first.units.Units.Volts;

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
  // private static DigitalInput botLimitSwitch;
  private static PIDController elevatorPID;
  private static RelativeEncoder elevatorEncoder;
  private static ElevatorFeedforward ff;
  // private boolean underBotSwitch; //in case we need it, threw it in commented sections. Might be needed because limit switch is not at the very
  //                                 // bottom of the elevator
  // private boolean previousSwitchTriggered;
  // private boolean currentSwitchTriggered;

  private final SysIdRoutine.Config sysIDConfig = new SysIdRoutine.Config(Volts.of(0.4).per(Seconds),  // we don't know what seconds does but it works (if there's errors then it may be because of this)
  Volts.of(2),
  Seconds.of(5),
   null);

  private final SysIdRoutine elevatorRoutine = new SysIdRoutine(
    sysIDConfig, 
    new SysIdRoutine.Mechanism(
      volts -> setVoltage(volts.in(Volts)), null, this)); 
   
  public Elevator() {
    elevatorMotorLeader = new SparkMax(Ports.ElevatorPorts.MOTOR_PORT, SparkLowLevel.MotorType.kBrushless );

    // botLimitSwitch = new DigitalInput(Ports.ElevatorPorts.BOT_SWITCH );

    elevatorEncoder = elevatorMotorLeader.getEncoder();

    elevatorPID = new PIDController(
      Constants.ElevatorConstants.PIDConstants.kP,
      Constants.ElevatorConstants.PIDConstants.kI,
      Constants.ElevatorConstants.PIDConstants.kD
    );
  
    ff = new ElevatorFeedforward(
      Constants.ElevatorConstants.FeedForwardConstants.kS, 
      Constants.ElevatorConstants.FeedForwardConstants.kG, 
      Constants.ElevatorConstants.FeedForwardConstants.kV
    );

      SparkMaxConfig config = new SparkMaxConfig();

        config
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        config.encoder
        .positionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(Constants.ElevatorConstants.PIDConstants.kP, Constants.ElevatorConstants.PIDConstants.kI, Constants.ElevatorConstants.PIDConstants.kD);
        //probably unneeded 

        elevatorMotorLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // underBotSwitch = true;
        // currentSwitchTriggered = botLimitSwitch.get();
        // previousSwitchTriggered = currentSwitchTriggered;

        
    }    
  
    /**
     * sets motor voltage using calculations from PID and FF values 
     */
    public void elevatorPID(double setpoint){
      // if (botLimitSwitch.get()) {
      //   elevatorMotorLeader.set(0);
      // }

      // else {
        elevatorMotorLeader.setVoltage(
          elevatorPID.calculate( elevatorEncoder.getPosition() ) + 
          ff.calculate( elevatorPID.calculate(elevatorEncoder.getPosition()) ) ); // not sure if this is correct
          // elevatorMotorFollower.resumeFollowerMode();
      // }
       
    }

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
    //  */
    // public void hitBotLimit(){
    //   if(botLimitSwitch.get()){
    //     stopMotorCmd();
    //   }
    // }

    // public boolean hitBotLimit() {
    //   return botLimitSwitch.get();
    // }

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
        System.out.println("FORWARD " + elevatorMotorLeader.getAppliedOutput());
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
          if(elevatorEncoder.getPosition() < ElevatorConstants.SetpointConstants.FIRST_LVL){
            elevatorMotorLeader.stopMotor();
          } else {
            elevatorMotorLeader.set(-Constants.ElevatorConstants.MOTOR_SPEED);
            System.out.println("reverseeeeeeee");
          }
        }
      );
    }

    public Command stopMotorCmd(){
      return this.run( () -> 
        elevatorMotorLeader.set(0)
      );
    }

    /**
     * 
     * @param setpoint
     * @return lifts elevator to specified setpoint
     */
    public Command setLevel(double setpoint) {
      return this.run(() -> elevatorPID(setpoint));
    }


    public void setVoltage(double volts){
      elevatorMotorLeader.setVoltage(volts);
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
  //  SmartDashboard.putBoolean("Bottom Limit Switch", hitBotLimit());
   SmartDashboard.putNumber("encoder", elevatorEncoder.getPosition());
  //  SmartDashboard.putBoolean("Under limit switch", underBotSwitch);
  //  botSwitchStatus();
  }
}
// }
