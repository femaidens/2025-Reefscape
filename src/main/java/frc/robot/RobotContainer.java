// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.auto.Taxi;
import frc.robot.auto.TaxiL2;
import frc.robot.auto.TaxiL3;
import frc.robot.auto.TaxiL4;
import frc.robot.commands.AlignToCenter;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralTransition;
import frc.robot.commands.Elevating;
// import frc.robot.commands.DriveToPoseCmd;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Vision;
import monologue.Logged;

import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Seconds;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {
    // The robot's subsystems and commands are defined here...
    // private final Drive drive = new Drive();
    private final Vision vision;
    private final Elevator elevator;
    //private final Intake intake;
    private final Outtake outtake;
    private final LED led; 

    private final Elevating elevating;
    //private final DriveSim driveSim;
    // private final Autos autos;
    private final CoralTransition coralTransition;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driveJoy = new CommandXboxController(OperatorConstants.DRIVER_PORT);
    private final CommandXboxController operJoy = new CommandXboxController(OperatorConstants.OPERATOR_PORT);
    private SendableChooser<Command> autonChooser;

    // private final AlignToCenter alignToCenter;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        vision = new Vision();
        elevator = new Elevator();
        outtake = new Outtake();
        led = new LED(); 
        elevating = new Elevating(elevator, outtake);
        coralTransition = new CoralTransition(outtake,led);
        autonChooser = new SendableChooser<>();
        // autos = new Autos (drivetrain, outtake, intake, elevator, coralTransition,
        // elevating);
        // alignToCenter = new AlignToCenter(drive, vision, null);
        configureBindings();
        configureDefaultCmds();
        configureAuton();
    }

    

    private void configureDefaultCmds() {
        vision.setDefaultCommand(
                new RunCommand(
                        () -> vision.driveFromVision(
                                () -> MathUtil.applyDeadband(driveJoy.getLeftY(), 0.1),
                                () -> MathUtil.applyDeadband(driveJoy.getLeftX(), 0.1),
                                () -> MathUtil.applyDeadband(driveJoy.getRightX(), 0.1)),
                        vision));

        // elevator.setDefaultCommand(
        //     elevator.stayAtLevel()
        // );
        

//                         driveSim.setDefaultCommand(
//       driveSim.drive(()-> -driveJoy.getLeftY(), ()-> -driveJoy.getLeftX(), () ->-driveJoy.getRightX()));

        // led.setDefaultCommand(
        //     led.setScrollGPCmd() 
        // );
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        driveJoy.rightBumper()
                .onTrue(vision.resetGyroFromVision());

        driveJoy.rightTrigger()
                .onTrue(vision.funkierRight())
                .onFalse(vision.stopDriving());

        driveJoy.leftTrigger()
                .onTrue(vision.funkierLeft())
                .onFalse(vision.stopDriving());

        driveJoy.y()
                .onTrue(vision.funkierMiddle())
                .onFalse(vision.stopDriving());

        operJoy.povUp()
                .whileTrue(elevator.runMotorCmd())
                .onFalse(elevator.stopMotorCmd()
                // .andThen(elevator.setCurrentSetpoint(elevator.getCurrentPosition()))
                );

        operJoy.povDown()
                .whileTrue(elevator.reverseMotorCmd())
                .onFalse(elevator.stopMotorCmd()
                // .andThen(elevator.setCurrentSetpoint(elevator.getCurrentPosition()))
                );

        /**
        * outtake
        */
        operJoy.rightBumper()
            .whileTrue(outtake.runMotorCmd().alongWith(led.setPinkCmd()))
            .onFalse(outtake.stopMotorCmd());

        /**
        * reverse outtake
        */
        operJoy.leftBumper()
            .whileTrue(outtake.reverseOuttakeCmd())
            .onFalse(outtake.stopMotorCmd());

        /**
         * coral transition
         */
        operJoy.rightTrigger()
            .onTrue(coralTransition.moveCoralToOuttake());

        operJoy.leftTrigger()
            .onTrue(outtake.stopMotorCmd()
            .andThen(elevating.resetDefault()));

        operJoy.a()
            .onTrue(elevating.secondLevelCmd());

        operJoy.b()
            .onTrue(elevating.thirdLevelCmd());
            
        operJoy.y()
            // .onTrue(elevating.possibleFourthLevelCmd());
            .onTrue(elevating.fourthLevelCmd());

        operJoy.x()
            .onTrue(elevating.scoringAlgaeBargeCmd());
            // .onFalse(outtake.stopMotorCmd());

        operJoy.start()
            .onTrue(elevating.algaeSecondLevelCmd());
            
        operJoy.back()
            .onTrue(elevating.algaeThirdLevelCmd());

        /**
         * elevator sysid
         */
        // driveJoy.a()
        // .whileTrue(
        //         elevator.quasiCmd(SysIdRoutine.Direction.kForward));

        // driveJoy.b()
        //         .whileTrue(
        //                 elevator.quasiCmd(SysIdRoutine.Direction.kReverse));

        // driveJoy.x()
        //         .whileTrue(
        //                 elevator.dynaCmd(SysIdRoutine.Direction.kForward));

        // driveJoy.y()
        //         .whileTrue(
        //                 elevator.dynaCmd(SysIdRoutine.Direction.kReverse));
    }

    public void configureAuton(){
        autonChooser.addOption("taxi", new Taxi(vision));
        autonChooser.addOption("taxi L2", new TaxiL2(elevating, outtake, vision, coralTransition));
        autonChooser.addOption("taxi L3", new TaxiL3(elevating, outtake, vision, coralTransition));
        autonChooser.addOption("taxi L4", new TaxiL4(elevating, outtake, vision, coralTransition));
        SmartDashboard.putData("Choose auto: ", autonChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autonChooser.getSelected();
    }

}