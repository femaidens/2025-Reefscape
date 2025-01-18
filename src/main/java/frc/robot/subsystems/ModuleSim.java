
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.ModuleSpark;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants;

public class ModuleSim{
    private DCMotorSim driveSim;

    private ModuleSpark motor;

    public double drivePositionRad;
    public double driveVelocityRadPerSec;
    public  double driveAppliedVolts; //don't change
    public double[] driveCurrentAmps;
    
    public double turnAbsolutePositionRad;
    public double turnRelativePositionRad;
    public double turnVelocityRadPerSec;
    public double turnAppliedVolts;
    public double[] turnCurrentAmps;

    public ModuleSim(){
        driveSim = new DCMotorSim(
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
    }


    // public void updateInputs(double drivePositionRad, double driveVelocityRadPerSec, double driveAppliedVolts, double driveCurrentAmps,
    // double turnAbsolutePositionRad, double turnRelativePositionRad, double turnVelocityRadPerSec, double turnAppliedVolts, double turnCurrentAmps){
    //     driveSim.update(Constants.ModuleSimConstants.loopPeriodSecs);
    //     turnSim.update(Constants.ModuleSimConstants.loopPeriodSecs);

    //     double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Constants.ModuleSimConstants.loopPeriodSecs;
    //     turnAbsolutePositionRad += angleDiffRad;
    //     turnRelativePositionRad += angleDiffRad;
        
    //     while (turnAbsolutePositionRad < 0){
    //         turnAbsolutePositionRad += 2.0 * Math.PI;
    //     }
    //     while (turnAbsolutePositionRad > 2.0 * Math.PI){
    //         turnAbsolutePositionRad -= 2.0 * Math.PI;
    //     }

    //     this.drivePositionRad = drivePositionRad + (driveSim.getAngularVelocityRadPerSec()*0);
    //     this.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    //     this.driveAppliedVolts = driveAppliedVolts;
    //     this.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    //     // this.driveTempCelcius = new double[] {};//inputs comes from climb inputs
    // }
    public void setDriveVoltage(double volts) {
        double v = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(v);
        driveSim.update(0.02);
    }

    // public void setTurnVoltage(double volts) {
    //     double v = MathUtil.clamp(volts, -12.0, 12.0);
    //     turnSim.setInputVoltage(v);
    // }

    public double getDriveAppliedVolts(){
        return driveAppliedVolts;
    }

       public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }
      public SwerveModuleState getState(){
        return new SwerveModuleState(5, 
            Rotation2d.fromDegrees(90));
    }
    public double getDrivePosition(){
        return 0;
    }

       /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    driveSim.setInput(motor.getDriveVelocity()/Constants.ModuleSimConstants.MAX_SPEED * RobotController.getBatteryVoltage());
    //updateInputs(drivePositionRad, driveVelocityRadPerSec, driveAppliedVolts, drivePositionRad, turnAbsolutePositionRad, turnRelativePositionRad, turnVelocityRadPerSec, turnAppliedVolts, driveAppliedVolts);

    // Next, we update it. The standard loop time is 20ms.
    driveSim.update(0.020);

    
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(driveSim.getCurrentDrawAmps()));
  }
}


