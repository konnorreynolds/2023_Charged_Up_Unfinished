// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */

  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_forwardSpeed;
  private final DoubleSupplier m_rotationSpeed;


  public ArcadeDrive(DriveTrain drivetrain, DoubleSupplier forwardSpeed, DoubleSupplier rotationSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    // Initialize command requirements
    m_drivetrain = drivetrain;
    m_forwardSpeed = forwardSpeed;
    m_rotationSpeed = rotationSpeed;

    addRequirements(m_drivetrain);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Drive the robot with arcadeDrive perameters
    m_drivetrain.arcadeDrive(m_forwardSpeed.getAsDouble(), m_rotationSpeed.getAsDouble());

    // Runs the baseSpeed subsytem command while ArcadeDrive command is active
    m_drivetrain.baseSpeed();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
