package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Translation.PID;
import frc.robot.subsystems.DriveConstants.Turn;


public class ModuleKraken {
    private final TalonFX driveMotor; 
    private final TalonFX turnMotor; 
    
    // This is both an absolute and relative encoder 
    private final CANcoder encoder;
    
    //PID? 
    private PhoenixPIDController drivePidController; 
    private PhoenixPIDController turnPidController; 

    
    // IDK what canbus we use so i just set it to the robot rio for now. 
    //THIS SHOULD BE SUBJECT TO CHANGE?! 
    public ModuleKraken(int driveID, int turnID){
        driveMotor = new TalonFX(driveID, "rio"); 
        configureTalon(driveMotor); 

        turnMotor = new TalonFX(turnID, "rio"); 
        configureTalon(turnMotor); 

        drivePidController = new PhoenixPIDController(Translation.PID.P, Translation.PID.I, Translation.PID.D); 
        turnPidController = new PhoenixPIDController(Turn.PID.P,Turn.PID.I, Turn.PID.D);

        encoder = new CANcoder(0, "rio"); 
        
         
    }

    /**
     * This configures a TalonFX motor!!! 
     * @param 
     */

     public static void configureTalon(TalonFX motor){
        motor.setNeutralMode(NeutralModeValue.Brake); 
        motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Translation.CURRENT_LIMIT)); 
     }

     



    public void setDriveVoltage(double volts){
        driveMotor.setVoltage(volts); 
    }

    public void setTurnVoltage(double volts){
        turnMotor.setVoltage(volts); 
    }



}
