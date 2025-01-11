package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.subsystems.DriveConstants.Translation;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor;

public class ModuleKraken {
    private final TalonFX driveMotor; 
    private final TalonFX turnMotor; 
    
    // I give up on the encoders for now :/ 
    //private final ParentDevice driveEncoder;
    //private final ParentDevice turnEncoder; 
    
    private final Slot0Configs slot0Turn; 
    private final Slot1Configs slot1Drive; 
    
    private final TalonFXConfiguration driveConfig; 
    private final CurrentLimitsConfigs driveCurrentLimit;
    private final TalonFXConfiguration turnConfig;
    private final CurrentLimitsConfigs turnCurrentLimit;  
    // IDK what canbus we use so i just set it to the robot rio for now. 
    //THIS SHOULD BE SUBJECT TO CHANGE?! 
    public ModuleKraken(int driveID, int turnID){
        driveMotor = new TalonFX(driveID, "rio"); 
        turnMotor = new TalonFX(turnID, "rio"); 

        // Highkey don't know how to do this...
        ///driveEncoder = new ParentDevice(driveID, "encoder", "rio"); 
        //turnEncoder.create(driveID, "rio");

        slot0Turn = new Slot0Configs().withKP(Translation.PID.P).withKI(Translation.PID.I).withKD(Translation.PID.D);
        slot1Drive = new Slot1Configs().withKP(0).withKI(0).withKD(0); 

        driveConfig = new TalonFXConfiguration();
        driveCurrentLimit = new CurrentLimitsConfigs();
        driveCurrentLimit.withStatorCurrentLimit(Translation.CURRENT_LIMIT);  
        driveConfig.withCurrentLimits(driveCurrentLimit); 
        
        turnConfig = new TalonFXConfiguration(); 
        turnCurrentLimit = new CurrentLimitsConfigs(); 
        turnCurrentLimit.withStatorCurrentLimit(Translation.CURRENT_LIMIT); 
        
    }



}
