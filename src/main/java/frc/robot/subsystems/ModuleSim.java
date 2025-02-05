
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ModuleSpark;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class ModuleSim {
    private DCMotorSim turnMotorSim;

    private DCMotorSim driveMotorSim;

    private PIDController drivePIDController;
    private PIDController turnPIDController;

    private SimpleMotorFeedforward driveFFController;
    public SwerveModuleState desiredState = new SwerveModuleState();

    public ModuleSim() {
        driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1.0 / Translation.POS_CONVERSION_FACTOR), DCMotor.getNEO(1),0.5, 0.5);
        turnMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.25, 1.0 / Turn.POS_CONVERSION_FACTOR), DCMotor.getNEO(1),0.5, 0.5);

        drivePIDController = new PIDController(Translation.PID.P, Translation.PID.I, Translation.PID.D);
        turnPIDController = new PIDController(Turn.PID.P, Turn.PID.I, Turn.PID.D);
        turnPIDController.enableContinuousInput(0, Math.PI * 2);
        driveFFController = new SimpleMotorFeedforward(Translation.FF.S, Translation.FF.V);
        turnPIDController.setTolerance(DriveSimConstants.TURNING_PID_POSITION_TOL, DriveSimConstants.TURNING_PID_VELOCITY_TOL);
        drivePIDController.setTolerance(DriveSimConstants.DRIVE_PID_POSITION_TOL, DriveSimConstants.DRIVE_PID_VELOCITY_TOL);
    }

    public void setDesiredState(SwerveModuleState state) {
        // state.optimize(state.angle);
        // System.out.println(state.angle);
        double voltage = driveFFController.calculate(state.speedMetersPerSecond)
                + drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        setDriveVoltage(voltage);
        // System.out.println(voltage);
        double turnVoltage = turnPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians());
        System.out.println(turnVoltage);
        setTurnVoltage(turnVoltage);
        desiredState = state;
    }

    public void setDriveVoltage(double volts) {
        // double v = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotorSim.setInputVoltage(volts);
        driveMotorSim.update(0.02);
    }

    public void setTurnVoltage(double volts) {
        double v = MathUtil.clamp(volts, -12.0, 12.0);
        turnMotorSim.setInputVoltage(v);
        turnMotorSim.update(0.02);
    }

    // public double getDriveAppliedVolts() {
    //     return driveAppliedVolts;
    // }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

    public double getTurnAngle() {
        return turnMotorSim.getAngularPositionRotations();//getAngularPositionRad();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                Rotation2d.fromRadians(getTurnAngle()));
    }

    public double getDrivePosition() {
        return driveMotorSim.getAngularPosition().in(Units.Revolutions) * Translation.POS_CONVERSION_FACTOR;
    }

    public double getDriveVelocity() {
        return driveMotorSim.getAngularVelocity().in(Units.RevolutionsPerSecond) * Translation.VEL_CONVERSION_FACTOR;
    }

    /** Advance the simulation. */
    public void simulationPeriodic() {
        // // In this method, we update our simulation of what our elevator is doing
        // // First, we set our "inputs" (voltages)
        // driveMotorSim.setInput(driveMotorSim.getDriveVelocity()/Constants.ModuleSimConstants.MAX_SPEED
        // * RobotController.getBatteryVoltage());
        // //updateInputs(drivePositionRad, driveVelocityRadPerSec, driveAppliedVolts,
        // drivePositionRad, turnAbsolutePositionRad, turnRelativePositionRad,
        // turnVelocityRadPerSec, turnAppliedVolts, driveAppliedVolts);
        // // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(driveMotorSim.getCurrentDrawAmps() + turnMotorSim.getCurrentDrawAmps()));

    }
}
