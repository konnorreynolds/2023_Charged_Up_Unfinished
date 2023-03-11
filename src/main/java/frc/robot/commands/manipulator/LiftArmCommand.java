// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class LiftArmCommand extends CommandBase {
  /** Creates a new LiftArmCommand. */
  private Manipulator _manipulator;
  private double _power;
  public LiftArmCommand(Manipulator manipulator, double liftPower) {
    // Use addRequirements() here to declare subsystem dependencies.
  this._manipulator = manipulator;
  this._power = liftPower;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._manipulator.raiseArm(this._power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._manipulator.stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
