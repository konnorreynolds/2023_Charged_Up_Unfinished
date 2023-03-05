// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Manipulator;

public class ExtendArm extends CommandBase {
  /** Creates a new ExtendArm. */

  private Manipulator _manipulator;
  private CommandJoystick _opJoystick;

  public ExtendArm(Manipulator manipulator, CommandJoystick opJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.

    // Initialize requirements
    this._manipulator = manipulator;
    this._opJoystick = opJoystick;

    addRequirements(manipulator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Controls intake extender with operator joysticks Y axis
    this._manipulator.extendArm(this._opJoystick.getRawAxis(1));

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
