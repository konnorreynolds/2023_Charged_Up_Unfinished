// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  AHRS navx = new AHRS(SPI.Port.kMXP);


  /* Speed control subsystem commands */
  //
  public void baseSpeed() {
    drive.setMaxOutput(.7);
  }
  public void boost() {
    drive.setMaxOutput(1);
  }
  public void slow() {
    drive.setMaxOutput(.30);
  }

  public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
    drive.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  // Subsystem command to set all drive motors to brake to be used in a toggleOnTrue command
  public void setIdleBrake() {
    rightMotorID0.setNeutralMode(NeutralMode.Brake);
    rightMotorID1.setNeutralMode(NeutralMode.Brake);
    rightTalonID2.setNeutralMode(NeutralMode.Brake);
    leftMotorID3.setNeutralMode(NeutralMode.Brake);
    leftMotorID4.setNeutralMode(NeutralMode.Brake);
    leftTalonID5.setNeutralMode(NeutralMode.Brake);
  }

  // Subsystem command to return all the drive motors to coast to be used in a toggleOnTrue command
  public void setIdleCoast() {
    rightMotorID0.setNeutralMode(NeutralMode.Coast);
    rightMotorID1.setNeutralMode(NeutralMode.Coast);
    rightTalonID2.setNeutralMode(NeutralMode.Coast);
    leftMotorID3.setNeutralMode(NeutralMode.Coast);
    leftMotorID4.setNeutralMode(NeutralMode.Coast);
    leftTalonID5.setNeutralMode(NeutralMode.Coast);
  }

  // Subsystem command to stop all drive motors
  public void stop() {
    rightMotorID0.stopMotor();
    rightMotorID1.stopMotor();
    rightTalonID2.stopMotor();
    leftMotorID3.stopMotor();
    leftMotorID4.stopMotor();
    leftTalonID5.stopMotor();
  }

  public double getYaw() {
    return navx.getYaw();
  }
  public double getPitch() {
    return navx.getPitch();
  }
  public double getRoll() {
    return navx.getRoll();
  }
  public double getAngle() {
    return navx.getAngle();
  }
  public void calibrateGyro() {
    navx.calibrate();
  }
  public void zeroGyro() {
    navx.reset();
    navx.zeroYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Yaw", navx.getYaw());
    SmartDashboard.putNumber("Pitch", navx.getPitch());
    SmartDashboard.putNumber("Roll", navx.getRoll());

    leftMotorID3.setInverted(true);
    leftMotorID4.setInverted(true);
    leftTalonID5.setInverted(true);
    
  }

}
