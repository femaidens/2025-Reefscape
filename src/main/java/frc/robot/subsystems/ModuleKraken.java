package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;


public class ModuleKraken {
    private final TalonFX driveMotor; 
    private final TalonFX turnMotor; 
    
    // I give up on the encoders for now :/ 
    //private final ParentDevice driveEncoder;
    //private final ParentDevice turnEncoder; 
    
    //PID? 
    private final Slot0Configs slot0Turn; 
    private final Slot1Configs slot1Drive; 
       

    // IDK what canbus we use so i just set it to the robot rio for now. 
    //THIS SHOULD BE SUBJECT TO CHANGE?! 
    public ModuleKraken(int driveID, int turnID){
        driveMotor = new TalonFX(driveID, "rio"); 
        driveMotor.setNeutralMode(NeutralModeValue.Brake); 
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Translation.CURRENT_LIMIT)); 


        turnMotor = new TalonFX(turnID, "rio"); 
        turnMotor.setNeutralMode(NeutralModeValue.Brake); 
        turnMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Turn.CURRENT_LIMIT)); 

        // Highkey don't know how to work the encoders broooooooooooooooooo...
        ///driveEncoder = new ParentDevice(driveID, "encoder", "rio"); 
        //turnEncoder.create(driveID, "rio");

        //This has all the PID values, but IDK how to calculate with them... erm 
        slot0Turn = new Slot0Configs().withKP(Translation.PID.P).withKI(Translation.PID.I).withKD(Translation.PID.D);
        slot1Drive = new Slot1Configs().withKP(Translation.PID.P).withKI(Translation.PID.I).withKD(Translation.PID.D); 

         
    }




    public void setDriveVoltage(double volts){
        driveMotor.setVoltage(volts); 
    }

    public void setTurnVoltage(double volts){
        turnMotor.setVoltage(volts); 
    }



}
