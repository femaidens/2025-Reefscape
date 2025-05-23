// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;

/** Add your docs here. */
public class ModuleSpark {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;

    private final double chassisAngularOffset;
    double angleSetpoint = 0;

    private final PIDController drivePIDController;
    private final SimpleMotorFeedforward driveFFController;
    private final PIDController turnPIDController;
    private final SimpleMotorFeedforward turnFFController;

    private SwerveModuleState desiredState = null;

    /**
     * Constructs a CANSpark module.
     * 
     * @param driveID              Drive motor ID Port
     * @param turnID               Turn motor ID Port
     * @param chassisAngularOffset In radians (ideally), Angular offset of module
     *                             (since they're all oriented in a different
     *                             direction)
     */
    public ModuleSpark(int driveID, int turnID, double chassisAngularOffset) {
        this.chassisAngularOffset = chassisAngularOffset;

        drivePIDController = new PIDController(Translation.PID.P, Translation.PID.I, Translation.PID.D);
        driveFFController = new SimpleMotorFeedforward(Translation.FF.S, Translation.FF.V);
        turnPIDController = new PIDController(Turn.PID.P, Turn.PID.I, Turn.PID.D);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        turnFFController = new SimpleMotorFeedforward(Turn.FF.S, Turn.FF.V);

        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder();

        driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(Translation.CURRENT_LIMIT);

        // encoder set up based on
        // https://github.com/wpilibsuite/2025Beta/discussions/27#discussioncomment-11522637
        driveConfig.encoder.positionConversionFactor(Translation.POS_CONVERSION_FACTOR);
        driveConfig.encoder.velocityConversionFactor(Translation.VEL_CONVERSION_FACTOR);
        driveConfig.encoder.inverted(false);

        turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(IdleMode.kBrake);
        turnConfig.smartCurrentLimit(Turn.CURRENT_LIMIT);

        turnConfig.absoluteEncoder.positionConversionFactor(Turn.POS_CONVERSION_FACTOR);
        turnConfig.absoluteEncoder.velocityConversionFactor(Turn.VEL_CONVERSION_FACTOR);
        turnConfig.absoluteEncoder.inverted(true);

        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        // for an explanation on why kNoPersistParameters:
        // https://github.com/wpilibsuite/2025Beta/discussions/27#discussioncomment-11217925
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(getState().angle);
        driveMotor.setVoltage(
                driveFFController.calculate(state.speedMetersPerSecond)
                        + drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
        turnMotor.setVoltage(turnPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians()));
        // SmartDashboard.putString("Swerve " + driveMotor.getDeviceId() + ":",
        // state.toString());
    }

    public void setDesiredStateNoPID(SwerveModuleState state) {
        // maybe optimize is broken
        SwerveModuleState newState = optimizeTest(
                new SwerveModuleState(state.speedMetersPerSecond,
                        new Rotation2d(Math.toRadians(state.angle.getRadians()))),
                new Rotation2d(Math.toRadians(getTurnAngle())));
        angleSetpoint = newState.angle.getRadians();
        driveMotor.setVoltage(driveFFController.calculate(state.speedMetersPerSecond));
        // going right breaks the frontleft motor but you can fix it bro!!! but I can't
        // fix any of the other ones
        turnMotor.setVoltage(turnPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve " + driveMotor.getDeviceId() + ":", state.toString());
        desiredState = state;
    }

    /**
     * Directly set voltage of drive motor in module
     * 
     * @param volts in volts
     */
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    /**
     * Directly set voltage of turn motor in module
     * 
     * @param volts
     */
    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }

    /**
     * Returns swerve state
     * 
     * @return SwerveModuleState (velocity and angle)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                new Rotation2d(getTurnAngle() - chassisAngularOffset));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

    /**
     * Returns the drive velocity
     * 
     * @return meters per second (ideally)
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity(); // * Translation.VEL_CONVERSION_FACTOR;
    }

    /**
     * Returns the turning velocity
     * 
     * @return radians per second (ideally)
     */
    public double getTurnVelocity() {
        return turnEncoder.getVelocity(); // * Turn.VEL_CONVERSION_FACTOR;
    }

    /**
     * Returns the position of drive motor
     * 
     * @return meters (ideally)
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition(); // * Translation.POS_CONVERSION_FACTOR;
    }

    /**
     * Returns the angle of turn motor
     * 
     * @return in radians (ideally)
     */
    public double getTurnAngle() {
        return turnEncoder.getPosition(); // * Turn.POS_CONVERSION_FACTOR;
    }

    /**
     * Resets the drive encoders
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    public static SwerveModuleState optimizeTest(SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getRadians()) == Math.PI) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    currentAngle);
        } else if (Math.abs(delta.getRadians()) > (3 * Math.PI) / 2) {
            return new SwerveModuleState(
                    desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromRadians(-(2 * Math.PI - delta.getRadians()))));
        } else if (Math.abs(delta.getRadians()) > Math.PI / 2) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromRadians(-(Math.PI - delta.getRadians()))));
        }

        else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

}