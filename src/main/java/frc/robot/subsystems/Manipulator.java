// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manuipulator. */

  private CANSparkMax rotationSpark = new CANSparkMax(ManipulatorConstants.rotationSpark, MotorType.kBrushless);
  private CANSparkMax extenderSpark = new CANSparkMax(ManipulatorConstants.extenderSpark, MotorType.kBrushless);

  private DoubleSolenoid pinchSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ManipulatorConstants.pinchSolenoidForward, ManipulatorConstants.pinchSolenoidReverse);

  static RelativeEncoder rotationEncoder;
  static RelativeEncoder extenderEncoder;

  public static SparkMaxPIDController rotationPID;
  public static SparkMaxPIDController extenderPID;

  public Manipulator() {
    rotationSpark.setIdleMode(IdleMode.kBrake);
    extenderSpark.setIdleMode(IdleMode.kBrake);

    rotationSpark.restoreFactoryDefaults();
    extenderSpark.restoreFactoryDefaults();

    rotationPID = rotationSpark.getPIDController();
    extenderPID = extenderSpark.getPIDController();

    rotationEncoder = rotationSpark.getEncoder();
    extenderEncoder = extenderSpark.getEncoder();

    rotationPID.setFeedbackDevice(rotationEncoder);
    extenderPID.setFeedbackDevice(extenderEncoder);

    rotationPID.setP(0);
    rotationPID.setI(0);
    rotationPID.setD(0);
    rotationPID.setIZone(0);
    rotationPID.setFF(0);
    rotationPID.setOutputRange(-1, 1);

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

  public void rotationSetpoint(double setpoint, double power) {

    if (rotationEncoder.getPosition() < setpoint) {
      rotationSpark.set(power);
    }
    else if (rotationEncoder.getPosition() > setpoint) {
      rotationSpark.set(-power);
    }
    /* 
    if (rotationEncoder.getPosition() >= setpoint) {
      rotationSpark.stopMotor();
    } */

  }

  public void extenderSetpoint(double setpoint, double power) {

    if (extenderEncoder.getPosition() < setpoint) {
      extenderSpark.set(power);
    }
    else if (extenderEncoder.getPosition() > setpoint) {
      extenderSpark.set(-power);
    }
    /* 
    if (extenderEncoder.getPosition() >= setpoint) {
      extenderSpark.stopMotor();
    }  */

  }

  public void raiseArm(double power) {
    rotationSpark.set(power);
  }

  public void lowerArm(double power) {
    rotationSpark.set(-power);
  }

  public void stopRotation() {
    rotationSpark.set(0);
  }

  public void extendArm(double power) {
    extenderSpark.set(power);
  }

  public void retractArm(double power) {
    extenderSpark.set(-power);
  }

  public void stopExtender() {
    extenderSpark.set(0);
  }

  public void pinchOpen() {
    pinchSolenoid.set(Value.kReverse);
  }

  public void pinchClose() {
    pinchSolenoid.set(Value.kForward);
  }

  public void pinchDefault() {
    pinchSolenoid.set(Value.kReverse);
  }

}
