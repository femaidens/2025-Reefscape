
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.DriveConstants.DriveSimConstants;
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
import edu.wpi.first.units.Units;

public class ModuleSim {
    private DCMotorSim turnMotorSim;

    private DCMotorSim driveMotorSim;

    private PIDController drivePIDController;
    private PIDController turnPIDController;

    private SimpleMotorFeedforward driveFFController;
    public SwerveModuleState desiredState = new SwerveModuleState();

    public ModuleSim() {
        driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.25, 1.0 / Translation.POS_CONVERSION_FACTOR), DCMotor.getKrakenX60(1));
        turnMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.1, 1.0), DCMotor.getKrakenX60(1));

        drivePIDController = new PIDController(Translation.PID.P, Translation.PID.I, Translation.PID.D);
        turnPIDController = new PIDController(Turn.PID.P, Turn.PID.I, Turn.PID.D);
        // turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        driveFFController = new SimpleMotorFeedforward(Translation.FF.S, Translation.FF.V);
        turnPIDController.setTolerance(DriveSimConstants.TURNING_PID_POSITION_TOL, DriveSimConstants.TURNING_PID_VELOCITY_TOL);
        drivePIDController.setTolerance(DriveSimConstants.DRIVE_PID_POSITION_TOL, DriveSimConstants.DRIVE_PID_VELOCITY_TOL);
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(state.angle);
        // System.out.println(state.angle);
        double voltage = driveFFController.calculate(state.speedMetersPerSecond)
                + drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        setDriveVoltage(voltage);
        // System.out.println(voltage);
        double turnVoltage = turnPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians());
        // System.out.println(turnVoltage);
        if(!isAtAngle(state)){
            setTurnVoltage(turnVoltage);
        }
        desiredState = state;
    }

    public boolean isAtAngle(SwerveModuleState state){
        return Math.abs(state.angle.getDegrees() - getState().angle.getDegrees()) < 10;
    }

    public void setDriveVoltage(double volts) {
        // double v = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotorSim.setInputVoltage(volts);
        driveMotorSim.update(0.02);
    }

    public void setTurnVoltage(double volts) {
        double v = MathUtil.clamp(volts, -14.0, 14.0);
        turnMotorSim.setInputVoltage(v);
        turnMotorSim.update(0.02);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

    public double getTurnAngle() {
        return (turnMotorSim.getAngularPositionRotations() * Turn.POS_CONVERSION_FACTOR) % (Math.PI * 2);  //getAngularPositionRad();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                Rotation2d.fromRadians(getTurnAngle()));
    }

    public double getDrivePosition() {
        return driveMotorSim.getAngularPositionRotations() * Translation.POS_CONVERSION_FACTOR;
    }

    public double getDriveVelocity() {
        return driveMotorSim.getAngularVelocityRPM() * Translation.VEL_CONVERSION_FACTOR;
    }

    

    /** Advance the simulation. */
    public void simulationPeriodic() {
        // // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(driveMotorSim.getCurrentDrawAmps() + turnMotorSim.getCurrentDrawAmps()));
                

    }
}
