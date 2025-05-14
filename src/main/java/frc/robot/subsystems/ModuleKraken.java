package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveConstants.Translation;
import frc.robot.subsystems.DriveConstants.Turn;
import monologue.Logged;
import monologue.Annotations.Log;

public class ModuleKraken implements Logged {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    // This is both an absolute and relative encoder
    // private final CANcoder driveEncoder;
    private final CANcoder turnEncoder; 
    //PID? 
    private final PIDController drivePIDController; 
    @Log.NT private final PIDController turnPIDController;
    
    private final SimpleMotorFeedforward driveFF;
    private final SimpleMotorFeedforward turnFF;

    private final MagnetSensorConfigs directionConfig;

    private final double chassisAngularOffset;

    private SwerveModuleState desiredState = null; 
    double angleSetpoint = 0;
    
    private double voltage; 

    public ModuleKraken(int driveID, int turnID, int CANCoderID, double magnetOffset, double chassisAngularOffset,
            boolean turnInverted) {
        this.chassisAngularOffset = chassisAngularOffset;

        driveMotor = new TalonFX(driveID, Translation.CANBUS);
        configureDriveTalon(driveMotor, Translation.CURRENT_LIMIT);

        turnMotor = new TalonFX(turnID, Translation.CANBUS);
        configureTurnTalon(turnMotor, CANCoderID, Turn.CURRENT_LIMIT, turnInverted);

        drivePIDController = new PIDController(Translation.PID.P, Translation.PID.I, Translation.PID.D); 
        turnPIDController = new PIDController(Turn.PID.P,Turn.PID.I, Turn.PID.D);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

        driveFF = new SimpleMotorFeedforward(Translation.FF.S, Translation.FF.V);
        turnFF = new SimpleMotorFeedforward(Turn.FF.S, Turn.FF.V);
        // DEVICE IDS SHOULD BE CHANGED!! 
        driveMotor.setPosition(0);  
        

        directionConfig = new MagnetSensorConfigs();
        turnEncoder = new CANcoder(CANCoderID, Translation.CANBUS);
        directionConfig.withAbsoluteSensorDiscontinuityPoint(0.5);
        directionConfig.MagnetOffset = magnetOffset;
        turnEncoder.getConfigurator().apply(directionConfig.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                new Rotation2d(getTurnAngle() - chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(getState().angle);
        driveMotor.setVoltage(
        driveFF.calculate(state.speedMetersPerSecond) + drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond));
        
        if(Math.abs(turnPIDController.getError()) < .2) {
            turnMotor.setVoltage(0);
        } else {
            turnMotor.setVoltage(turnPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians()));
        }
        desiredState = state;
    }

    public void setDesiredStateNoPID(SwerveModuleState state) {
        state.optimize(getState().angle);
        angleSetpoint = state.angle.getRadians();
        driveMotor.setVoltage(driveFF.calculate(state.speedMetersPerSecond));
        // going right breaks the frontleft motor but you can fix it bro!!! but I can't fix any of the other ones
        voltage = turnPIDController.calculate(getState().angle.getRadians(), angleSetpoint);
        turnMotor.setVoltage(voltage);
        SmartDashboard.putString("Swerve " + driveMotor.getDeviceID() + ":", state.toString());
        desiredState = state;
    }

    public double getVoltage(){
        return voltage;
    }

    public double getAbsolute(){
        return turnEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * This configures a TalonFX motor's neutral mode to brake and sets the current
     * limit
     * 
     * @param 
     */
    public static void configureDriveTalon(TalonFX motor, int currentLimit) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // config.Feedback.SensorToMechanismRatio = 1 / Translation.POS_CONVERSION_FACTOR;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(config);
    }

    /**
     * Configures turning TalonFX motor (sensor ratio, current limit, brake)
     * @param motor
     * @param encoderID
     * @param currentLimit
     * @param inverted
     */
    public static void configureTurnTalon(TalonFX motor, int encoderID, int currentLimit, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackRemoteSensorID = encoderID;
        config.Feedback.SensorToMechanismRatio = 1.0; // / Translation.POS_CONVERSION_FACTOR; // cancoder seems to be
                                                      // attached outside of gearbox
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : IN;

        // motor.setNeutralMode(NeutralModeValue.Brake); 
        // motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(currentLimit));
        // motor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(Translation.POS_CONVERSION_FACTOR));

        if(inverted){
        config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        }
        else{
            config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        }

        motor.getConfigurator().apply(config);  
    }

    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), getState().angle);
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity().getValueAsDouble() * Turn.VEL_CONVERSION_FACTOR;
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * Translation.VEL_CONVERSION_FACTOR;
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * Translation.POS_CONVERSION_FACTOR;
    }

    public double getTurnAngle() {
        return turnEncoder.getAbsolutePosition().getValueAsDouble() / Turn.POS_CONVERSION_FACTOR;
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
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