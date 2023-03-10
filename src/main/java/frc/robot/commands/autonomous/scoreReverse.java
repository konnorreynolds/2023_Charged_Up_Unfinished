// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.DriveForTime;
import frc.robot.commands.manipulator.LiftArmForTime;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreReverse extends SequentialCommandGroup {
  /** Creates a new scoreReverse. */
  public scoreReverse(DriveTrain drivetrain, Manipulator manipulator, CommandXboxController drController, double power, double liftPower, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LiftArmForTime(manipulator, -liftPower),
      Commands.runOnce(manipulator::pinchTrue, manipulator),
      new DriveForTime(drivetrain, power, time, drController)
    );
  }
}
