package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Translation.FF;
import frc.robot.subsystems.DriveConstants.Translation.PID;
import frc.robot.subsystems.DriveConstants.Turn;


public class ModuleKraken {
    private final TalonFX driveMotor; 
    private final TalonFX turnMotor; 
    
    // This is both an absolute and relative encoder 
    private final CANcoder driveEncoder;
    private final CANcoder turnEncoder; 
    //PID? 
    private final PhoenixPIDController drivePIDController; 
    private final PhoenixPIDController turnPIDController;
    
    private final SimpleMotorFeedforward driveFF;
    private final SimpleMotorFeedforward turnFF; 

    private final double chassisAngularOffset;
    private SwerveModuleState desiredState = null; 
        double angleSetpoint = 0; 
            // IDK what canbus we use so i just set it to the robot rio for now. 
            //THIS SHOULD BE SUBJECT TO CHANGE!!!! 
            public ModuleKraken(int driveID, int turnID, double chassisAngularOffset){
                this.chassisAngularOffset = chassisAngularOffset; 
                

                // NOTE: LOWKEY IDK HOW TO DO CONVERSIONS BC TALONS SEEM TO DO MOST OF THE STUFF IN ROTATIONS
                // SO RN EVERYTHING IS IN ROTATIONS THAT HAS BEEN CONVERTED TO A DOUBLE VALUE   
                driveMotor = new TalonFX(driveID, Translation.CANBUS); 
                configureTalon(driveMotor, Translation.CURRENT_LIMIT); 
        
                turnMotor = new TalonFX(turnID, Translation.CANBUS); 
                configureTalon(turnMotor,Turn.CURRENT_LIMIT); 
        
                drivePIDController = new PhoenixPIDController(Translation.PID.P, Translation.PID.I, Translation.PID.D); 
                turnPIDController = new PhoenixPIDController(Turn.PID.P,Turn.PID.I, Turn.PID.D);
    
                driveFF = new SimpleMotorFeedforward(Translation.FF.S,Translation.FF.V); 
                turnFF = new SimpleMotorFeedforward(Turn.FF.S, Turn.FF.V);
                // DEVICE IDS SHOULD BE CHANGED!! 
                driveEncoder = new CANcoder(0, Translation.CANBUS); 
                turnEncoder = new CANcoder(0,  Translation.CANBUS); 
                 
            }
        
        
            public SwerveModuleState getState(){
                return new SwerveModuleState(getDriveVelocity(),
                            new Rotation2d(getTurnAngle()-chassisAngularOffset));
            }
        
            public void setDesiredState(SwerveModuleState state){
                state.optimize(state.angle);
                driveMotor.setVoltage(
                driveFF.calculate(state.speedMetersPerSecond) + drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond,0.1)
            );
            turnMotor.setVoltage(turnPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians(),0.1));
        }
        public void setDesiredStateNoPID(SwerveModuleState state){
            // maybe optimize is broken 
            SwerveModuleState newState = optimizeTest(new SwerveModuleState(state.speedMetersPerSecond, new Rotation2d(Math.toRadians(state.angle.getRadians()))), new Rotation2d(Math.toRadians(getTurnAngle())));
            angleSetpoint = newState.angle.getRadians();
            driveMotor.setVoltage(driveFF.calculate(state.speedMetersPerSecond));
            // going right breaks the frontleft motor but you can fix it bro!!! but I can't fix any of the other ones 
            turnMotor.setVoltage(turnPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians(),0.1));
            SmartDashboard.putString("Swerve " + driveMotor.getDeviceID() + ":", state.toString());
            desiredState = state;
    }


    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), getState().angle); 
    }

    /**
     * This configures a TalonFX motor's nuetral mode to brake and sets the current limit!!! 
     * @param 
     */

     public static void configureTalon(TalonFX motor, int currentLimit){
        motor.setNeutralMode(NeutralModeValue.Brake); 
        motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(currentLimit)); 
     }

    

    public void setDriveVoltage(double volts){
        driveMotor.setVoltage(volts); 
    }

    public void setTurnVoltage(double volts){
        turnMotor.setVoltage(volts); 
    }
   
    // STILL HAVE TO DO THE CONVERSION FACTORS AND STUFF 
  
    public double getTurnVelocity(){
        return turnEncoder.getVelocity().getValueAsDouble(); 
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity().getValueAsDouble(); 
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition().getValueAsDouble(); 
    }

    
    //HELP IDK HOW TO DO CONVERSIONS BRUH 
    public double  getTurnAngle(){
        return turnEncoder.getAbsolutePosition().getValueAsDouble(); 
    }
    
    public void resetEncoders(){
        driveEncoder.setPosition(0); 
    }

    public static SwerveModuleState optimizeTest(SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
         if(Math.abs(delta.getRadians()) == Math.PI){
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                currentAngle);
         }
         else if (Math.abs(delta.getRadians()) > (3*Math.PI)/2) {
            return new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(Rotation2d.fromRadians(-(2*Math.PI - delta.getRadians()))));
          }
        else if (Math.abs(delta.getRadians()) > Math.PI/2) {
          return new SwerveModuleState(
              -desiredState.speedMetersPerSecond,
              desiredState.angle.rotateBy(Rotation2d.fromRadians(-(Math.PI - delta.getRadians()))));
        } 

        else {
          return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
      }

}
