
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ModuleSpark;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class ModuleSim{
    private DCMotorSim turnMotorSim;

    private DCMotorSim driveMotorSim;


    public double drivePositionRad;
    public double driveVelocityRadPerSec;
    public double driveAppliedVolts; //don't change
    public double[] driveCurrentAmps;
    
    public double turnAbsolutePositionRad;
    public double turnRelativePositionRad;
    public double turnVelocityRadPerSec;
    public double turnAppliedVolts;
    public double[] turnCurrentAmps;

    private PIDController drivePIDController;
    private PIDController turnPIDController;

    private SimpleMotorFeedforward driveFFController;
    public SwerveModuleState desiredState = new SwerveModuleState();


    public ModuleSim(){
        driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1),1, .5), DCMotor.getNEO(1),1, 1);
        turnMotorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1),1, .5), DCMotor.getNEO(1),1, 1);
    
       // motor = new ModuleSpark(164 , 9, 3);
    
        drivePositionRad = 3.0;
        driveVelocityRadPerSec = 5.0;
        driveAppliedVolts = 12.0; 
        driveCurrentAmps = new double[]{1, 2, 3, 4};

        turnAbsolutePositionRad = 2.0;
        turnRelativePositionRad = 3.0;
        turnVelocityRadPerSec = 5.0;
        turnAppliedVolts = 6.0;
        turnCurrentAmps = new double[]{2, 3, 4, 5};

        drivePIDController = new PIDController(Translation.PID.P, Translation.PID.I, Translation.PID.D);
        turnPIDController = new PIDController(Turn.PID.P, Turn.PID.I, Turn.PID.D);
        driveFFController = new SimpleMotorFeedforward(Translation.FF.S, Translation.FF.V);

 
    }

   
    public void setDesiredState(SwerveModuleState state){
        //possily change this optimize, since its differezxcvewasdwasdwasadwasdwasdawnt.
        //state.optimize(state.angle);
        double voltage = driveFFController.calculate(state.speedMetersPerSecond) + drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

        driveMotorSim.setInputVoltage(voltage); 
        System.out.println(voltage);
        turnMotorSim.setInputVoltage(turnPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians()));
        // SmartDashboard.putString("Swerve " + driveMotorSim.getDeviceId() + ":", state.toString());
        desiredState = state;
    }
    public void setDriveVoltage(double volts) {
        double v = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotorSim.setInputVoltage(v);
        driveMotorSim.update(0.02);
    }

    public void setTurnVoltage(double volts) {
        double v = MathUtil.clamp(volts, -12.0, 12.0);
        turnMotorSim.setInputVoltage(v);
        turnMotorSim.update(0.02);
    }

    public double getDriveAppliedVolts(){
        return driveAppliedVolts;
    }

    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

    public double getTurnAngle(){
        return turnMotorSim.getAngularPositionRad();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), 
            Rotation2d.fromRadians(getTurnAngle()));
    }

    public double getDrivePosition(){
        return 0;
    }

    public double getDriveVelocity(){
        return driveMotorSim.getAngularVelocity().in(Units.RadiansPerSecond)*2; // * Translation.VEL_CONVERSION_FACTOR;
    }


       /** Advance the simulation. */
    public void simulationPeriodic() {
    //     // In this method, we update our simulation of what our elevator is doing
    //     // First, we set our "inputs" (voltages)
    //     driveMotorSim.setInput(driveMotorSim.getDriveVelocity()/Constants.ModuleSimConstants.MAX_SPEED * RobotController.getBatteryVoltage());
    // //updateInputs(drivePositionRad, driveVelocityRadPerSec, driveAppliedVolts, drivePositionRad, turnAbsolutePositionRad, turnRelativePositionRad, turnVelocityRadPerSec, turnAppliedVolts, driveAppliedVolts);

    // // Next, we update it. The standard loop time is 20ms.
        driveMotorSim.update(0.020);
        turnMotorSim.update(0.020);
    // // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(driveMotorSim.getCurrentDrawAmps()));
        

  }
}


