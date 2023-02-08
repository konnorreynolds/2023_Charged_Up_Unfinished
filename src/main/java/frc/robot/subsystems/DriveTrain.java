// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private WPI_VictorSPX rightMotorID0 = new WPI_VictorSPX(DriveConstants.rightVictorID0);
  private WPI_VictorSPX rightMotorID1 = new WPI_VictorSPX(DriveConstants.rightVictorID1);
  private WPI_TalonSRX rightTalonID2 = new WPI_TalonSRX(DriveConstants.rightTalonID2);
  private MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorID0, rightMotorID1, rightTalonID2);
  private WPI_VictorSPX leftMotorID3 = new WPI_VictorSPX(DriveConstants.leftVictorID3);
  private WPI_VictorSPX leftMotorID4 = new WPI_VictorSPX(DriveConstants.leftVictorID4);
  private WPI_TalonSRX leftTalonID5 = new WPI_TalonSRX(DriveConstants.leftTalonID5);
  private MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotorID3, leftMotorID4, leftTalonID5);
  private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  

  public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
    drive.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  public void baseSpeed() {
    drive.setMaxOutput(.75);
  }
  public void boost() {
    drive.setMaxOutput(1);
  }
  public void slow() {
    drive.setMaxOutput(.5);
  }

  public void setIdleBrake() {
    rightMotorID0.setNeutralMode(NeutralMode.Brake);
    rightMotorID1.setNeutralMode(NeutralMode.Brake);
    rightTalonID2.setNeutralMode(NeutralMode.Brake);
    leftMotorID3.setNeutralMode(NeutralMode.Brake);
    leftMotorID4.setNeutralMode(NeutralMode.Brake);
    leftTalonID5.setNeutralMode(NeutralMode.Brake);
  }

  public void setIdleCoast() {
    rightMotorID0.setNeutralMode(NeutralMode.Coast);
    rightMotorID1.setNeutralMode(NeutralMode.Coast);
    rightTalonID2.setNeutralMode(NeutralMode.Coast);
    leftMotorID3.setNeutralMode(NeutralMode.Coast);
    leftMotorID4.setNeutralMode(NeutralMode.Coast);
    leftTalonID5.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }

}
