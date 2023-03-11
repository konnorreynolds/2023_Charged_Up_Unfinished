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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manuipulator. */

  // Initializes the Spark Max controllers for the NEOs 
  private static CANSparkMax liftSpark = new CANSparkMax(ManipulatorConstants.liftSpark, MotorType.kBrushless);
  private CANSparkMax extenderSpark = new CANSparkMax(ManipulatorConstants.extenderSpark, MotorType.kBrushless);

  // Initialized the solenoid for the intake pinch
  private Solenoid pinchSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  // Initializing the encoders for the sparks
  RelativeEncoder liftEncoder;
  RelativeEncoder extenderEncoder;

  // Initializing the PID controllers for the Sparks
  public static SparkMaxPIDController liftPID;
  public static SparkMaxPIDController extenderPID;

  public Manipulator() {
    // Setting the Idle mode to brake so the arm and extender don't move when not wanted
    liftSpark.setIdleMode(IdleMode.kBrake);
    extenderSpark.setIdleMode(IdleMode.kBrake);

    // Resetting the encoders and PID loop in the Sparks
    liftSpark.restoreFactoryDefaults();
    extenderSpark.restoreFactoryDefaults();

    // Getting the PID controllers for the Sparks
    liftPID = liftSpark.getPIDController();
    extenderPID = extenderSpark.getPIDController();

    // Getting the encoders for the sparks
    liftEncoder = liftSpark.getEncoder();
    extenderEncoder = extenderSpark.getEncoder();

    // Setting the encoders as a feedback device for the PID Loop
    liftPID.setFeedbackDevice(liftEncoder);
    extenderPID.setFeedbackDevice(extenderEncoder);

    // Intake lift PID loop initialization
    liftPID.setP(0);
    liftPID.setI(0);
    liftPID.setD(0);
    liftPID.setIZone(0);
    liftPID.setFF(0);
    liftPID.setOutputRange(-1, 1);

    // Intake extender PID loop initialization
    extenderPID.setP(0);
    extenderPID.setI(0);
    extenderPID.setD(0);
    extenderPID.setIZone(0);
    extenderPID.setFF(0);
    extenderPID.setOutputRange(-1, 1);
  }

  public void robotInit() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Puts encoder data onto the dashboard
    SmartDashboard.putNumber("Lift Position", liftEncoder.getPosition());
    SmartDashboard.putNumber("Extender Position", extenderEncoder.getPosition());

  }

  // WIP Intake lift setpoint subsystem command
  public void liftSetpoint(double setpoint, double power) {
    liftSpark.set(power);

    if (liftEncoder.getPosition() >= setpoint) {
      liftSpark.stopMotor();
    }

  }

  // WIP Intake extender setpoint subsystem command
  public void extenderSetpoint(double setpoint) {
    extenderPID.setReference(setpoint, ControlType.kPosition);
  }

  // Intake lift raise subsystem command
  public void raiseArm(double power) {
    liftSpark.set(power);
  }

  // Intake lift lower subsystem command
  public void lowerArm(double power) {
    liftSpark.set(-power);
  }

  // Intake lift stop subsystem command
  public void stopLift() {
    liftSpark.stopMotor();
  }

  // Intake extender extension subsystem command
  public void extendArm(double power) {
    extenderSpark.set(power / 2);
  }

  // Intake extender retract subsystem command
  public void retractArm(double power) {
    extenderSpark.set(-power / 2);
  }

  // Intake extender stop subsystem command
  public void stopExtender() {
    extenderSpark.stopMotor();
  }

  // Intake pinch subsystem command
  public void pinchTrue() {
    pinchSolenoid.set(true);
  }
  public void pinchFalse() {
    pinchSolenoid.set(false);
  }

  public void resetLiftEncoder() {
    liftEncoder.setPosition(0);
  }

}
