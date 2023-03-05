// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manuipulator. */

  // Initializes the Spark Max controllers for the NEOs 
  private CANSparkMax rotationSpark = new CANSparkMax(ManipulatorConstants.rotationSpark, MotorType.kBrushless);
  private CANSparkMax extenderSpark = new CANSparkMax(ManipulatorConstants.extenderSpark, MotorType.kBrushless);

  // Initialized the solenoid for the intake pinch
  private Solenoid pinchSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  // Initializing the encoders for the sparks
  static RelativeEncoder rotationEncoder;
  static RelativeEncoder extenderEncoder;

  // Initializing the PID controllers for the Sparks
  public static SparkMaxPIDController rotationPID;
  public static SparkMaxPIDController extenderPID;

  public Manipulator() {
    // Setting the Idle mode to brake so the arm and extender don't move when not wanted
    rotationSpark.setIdleMode(IdleMode.kBrake);
    extenderSpark.setIdleMode(IdleMode.kBrake);

    // Resetting the encoders and PID loop in the Sparks
    rotationSpark.restoreFactoryDefaults();
    extenderSpark.restoreFactoryDefaults();

    // Getting the PID controllers for the Sparks
    rotationPID = rotationSpark.getPIDController();
    extenderPID = extenderSpark.getPIDController();

    // Getting the encoders for the sparks
    rotationEncoder = rotationSpark.getEncoder();
    extenderEncoder = extenderSpark.getEncoder();

    // Setting the encoders as a feedback device for the PID Loop
    rotationPID.setFeedbackDevice(rotationEncoder);
    extenderPID.setFeedbackDevice(extenderEncoder);

    // Intake lift PID loop initialization
    rotationPID.setP(0);
    rotationPID.setI(0);
    rotationPID.setD(0);
    rotationPID.setIZone(0);
    rotationPID.setFF(0);
    rotationPID.setOutputRange(-1, 1);

    // Intake extender PID loop initialization
    extenderPID.setP(0);
    extenderPID.setI(0);
    extenderPID.setD(0);
    extenderPID.setIZone(0);
    extenderPID.setFF(0);
    extenderPID.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // WIP Intake lift setpoint subsystem command
  public void rotationSetpoint(double setpoint) {
    rotationPID.setReference(setpoint, ControlType.kPosition);
  }

  // WIP Intake extender setpoint subsystem command
  public void extenderSetpoint(double setpoint) {
    extenderPID.setReference(setpoint, ControlType.kPosition);
  }

  // Intake lift raise subsystem command
  public void raiseArm(double power) {
    rotationSpark.set(power);
  }

  // Intake lift lower subsystem command
  public void lowerArm(double power) {
    rotationSpark.set(-power);
  }

  // Intake lift stop subsystem command
  public void stopRotation() {
    rotationSpark.set(0);
  }

  // Intake extender extension subsystem command
  public void extendArm(double power) {
    extenderSpark.set(power);
  }

  // Intake extender retract subsystem command
  public void retractArm(double power) {
    extenderSpark.set(-power);
  }

  // Intake extender stop subsystem command
  public void stopExtender() {
    extenderSpark.set(0);
  }

  // Intake pinch subsystem command
  public void pinchToggle() {
    pinchSolenoid.toggle();
  }

}
