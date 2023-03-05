// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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

  // Vision
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final static double ANGULAR_P = 0.04;
  final static double ANGULAR_D = 0.0;
  public static PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    private double _targetYaw;
    private boolean _hasTarget;

    public void visionTurn(DoubleSupplier speedSupplier, boolean hasTarget, double DEADZONE, double targetYaw) {
      double joyStickSpeed = speedSupplier.getAsDouble();
      double speed;
      this._hasTarget = hasTarget;
      this._targetYaw = targetYaw;
      
      if (this._hasTarget){
        speed = turnController.calculate(this._targetYaw, 0);
      }
      else{
        //deadzone clause, deadzone is 0.12
        if(Math.abs(joyStickSpeed) > DEADZONE) {
          speed = joyStickSpeed*.75;
        }
        else {
          speed = 0;
        }
      }
  
      drive.setMaxOutput(speed);
    }

  public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
    drive.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  /* Speed control subsystem commands */
  // 
  public void baseSpeed() {
    drive.setMaxOutput(.6);
  }
  public void boost() {
    drive.setMaxOutput(1);
  }
  public void slow() {
    drive.setMaxOutput(.30);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }

}
