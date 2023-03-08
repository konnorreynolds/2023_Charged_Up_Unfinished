// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.BallVision;

public class AimAtBall extends CommandBase {
  /** Creates a new AimAtBall. */

  private DriveTrain _driveTrain;
  private CommandXboxController _controller;
  //private Joystick _leftController;
  private PIDController turnController;
  private BallVision _ballVision;
  private double _targetYaw;
  public static final double DEADZONE = 0.12;

  
  public AimAtBall(DriveTrain driveTrain, BallVision ballVision, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.

    turnController = DriveTrain.turnController;

    this._driveTrain = driveTrain;
    this._ballVision = ballVision;
    this._controller = controller;
    // this._leftController = leftController;

    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (this._controller.getRawAxis(6) > 10) {
      if (this._ballVision.hasTargets()) {
      // get yaw from camera
      this._targetYaw = this._ballVision.getYawVal();
      // trigger drive to turn to vision

        double rotationSpeed = turnController.calculate(this._targetYaw, -1);

      this._driveTrain.arcadeDrive(this._controller.getRawAxis(4), rotationSpeed);
     // else below is when no target is in place
    } 
  }
  // arcadeDrive(forwardSpeed, rotationSpeed)
    else {
      this._driveTrain.arcadeDrive(this._controller.getRawAxis(4), this._controller.getRawAxis(1));

      if (this._controller.leftBumper().getAsBoolean()) {
        this._driveTrain.slow();
      }
      else if (this._controller.rightBumper().getAsBoolean()) {
        this._driveTrain.boost();
      }
      else {
        this._driveTrain.baseSpeed();
      }

    }

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
