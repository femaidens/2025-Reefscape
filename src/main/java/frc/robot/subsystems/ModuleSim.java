
    /*package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Module;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

public class ModuleSim{
    private FlywheelSim driveSim;
    private FlywheelSim turnSim;

    // public double drivePositionRad;
    // public double driveVelocityRadPerSec;
    public double driveAppliedVolts;
    // public double[] driveCurrentAmps;
    
    public double turnAbsolutePositionRad;
    public double turnRelativePositionRad;
    // public double turnVelocityRadPerSec;
    public double turnAppliedVolts;
    // public double[] turnCurrentAmps;

    public ModuleSim(){
        driveSim = new FlywheelSim(DCMotor.getNEO(0), 0, 0);
        turnSim = new FlywheelSim(DCMotor.getNEO(0), 0, 0);
    
        // drivePositionRad = 0.0;
        // driveVelocityRadPerSec = 0.0;
        driveAppliedVolts = 0.0;
        // driveCurrentAmps = new double[4];

        turnAbsolutePositionRad = 0.0;
        turnRelativePositionRad = 0.0;
        // turnVelocityRadPerSec = 0.0;
        turnAppliedVolts = 0.0;
        // turnCurrentAmps = new double[4];
    }


    public void updateInputs(double drivePositionRad, double driveVelocityRadPerSec, double driveAppliedVolts, double driveCurrentAmps,
    double turnAbsolutePositionRad, double turnRelativePositionRad, double turnVelocityRadPerSec, double turnAppliedVolts, double turnCurrentAmps){
        driveSim.update(Constants.ModuleSimConstants.loopPeriodSecs);
        turnSim.update(Constants.ModuleSimConstants.loopPeriodSecs);

        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Constants.ModuleSimConstants.loopPeriodSecs;
        turnAbsolutePositionRad += angleDiffRad;
        turnRelativePositionRad += angleDiffRad;
        
        while (turnAbsolutePositionRad < 0){
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI){
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        this.drivePositionRad = drivePositionRad + (driveSim.getAngularVelocityRadPerSec()*0);
        this.driveVelocityRadPerSec driveSim.getAngularVelocityRadPerSec();
        this.driveAppliedVolts = driveAppliedVolts;
        this.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
        this.driveTempCelcius = new double[] {};//inputs comes from climb inputs

    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

}
*/

