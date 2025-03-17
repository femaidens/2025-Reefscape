// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Camera extends SubsystemBase {
//   private NetworkTable table;
//   /** Creates a new Camera. */
//   private ShuffleboardTab match = Shuffleboard.getTab("REEFSCAPE");
//   public Camera() {
//     match.addCamera("Camera stream", "deadfish", "mjpg:http://10.22.65.12:1181/?action=stream");
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
