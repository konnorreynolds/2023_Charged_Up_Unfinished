// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {

  public static CommandXboxController xboxController = new CommandXboxController(OIConstants.xboxController);
  public static DriveTrain m_drivetrain = new DriveTrain();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
      new ArcadeDrive(m_drivetrain, () -> xboxController.getRawAxis(4), () -> xboxController.getRawAxis(1))
      );

    xboxController.leftBumper().whileTrue(Commands.run(m_drivetrain::slow));
    xboxController.rightBumper().whileTrue(Commands.run(m_drivetrain::boost));
    xboxController.a().toggleOnTrue(Commands.startEnd(m_drivetrain::setIdleBrake, m_drivetrain::setIdleCoast));
  }

  public CommandXboxController getXboxController(){
    return xboxController;
}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopPeriodic() {
  }
  

}
