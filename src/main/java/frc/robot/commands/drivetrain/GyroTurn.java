// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GyroTurn extends CommandBase {
  /** Creates a new GyroTurn. */

  private final DriveTrain m_drivetrain;
  private final int m_angle;
  private PIDController turnController;
  AHRS gyro;

  public GyroTurn(DriveTrain drivetrain, int angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrain = drivetrain;
    m_angle = angle;

    gyro = new AHRS(SPI.Port.kMXP);

    addRequirements(drivetrain);

    SmartDashboard.putData("Gyro", gyro);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotationSpeed = turnController.calculate(gyro.getAngle(), m_angle);

    m_drivetrain.arcadeDrive(0, rotationSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
