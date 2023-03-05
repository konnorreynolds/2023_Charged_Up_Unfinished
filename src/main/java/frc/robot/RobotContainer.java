// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autonomous.scoreReverse;
import frc.robot.commands.drivetrain.DriveForTime;
import frc.robot.commands.manipulator.ExtendArm;
import frc.robot.commands.vision.AimAtBall;
import frc.robot.subsystems.BallVision;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;

public class RobotContainer {


  // Initializes controllers and subsystems for command mapping uses
  public static CommandXboxController drController = new CommandXboxController(OIConstants.drController);
  public static CommandJoystick opJoystick = new CommandJoystick(OIConstants.opJoystick);
  public static DriveTrain m_drivetrain = new DriveTrain();
  public static Manipulator m_manipulator = new Manipulator();
  public static BallVision m_ballvision = new BallVision();

  // Initializes autonomous chooser for Smart Dashboard
  private SendableChooser<Command> autonChooser = new SendableChooser<>();
  
  // Creates all the autonomous commands
  private final scoreReverse m_ScoreReverse = new scoreReverse(
    m_manipulator, m_drivetrain, .5, 3);
  private final DriveForTime m_Reverse = new DriveForTime(
    m_drivetrain, .5, 3);

  public RobotContainer() {
  
  // Autonomous chooser for Smart Dashboard
  autonChooser.setDefaultOption("scoreReverse", m_ScoreReverse);
  autonChooser.addOption("Reverse", m_Reverse);

  // Puts the autonomous chooser on the dashboard
  SmartDashboard.putData(autonChooser);

    configureBindings();
  }

  private void configureBindings() {

    /* Drivetrain Command to drive the robot */
    m_drivetrain.setDefaultCommand(new AimAtBall(m_drivetrain, m_ballvision, drController));

    /* ExtendArm default command to extend the arm with Joystick axis */
    m_manipulator.setDefaultCommand(new ExtendArm(m_manipulator, opJoystick));

    /* Driver Controller Inputs */
    // Speed Controls
    drController.leftBumper().whileTrue(Commands.run(m_drivetrain::slow, m_drivetrain));
    drController.rightBumper().whileTrue(Commands.run(m_drivetrain::boost, m_drivetrain));
    // Coast/Brake Control
    drController.start().toggleOnTrue(Commands.startEnd(m_drivetrain::setIdleBrake, m_drivetrain::setIdleCoast));
    // Vision WIP
    drController.a().toggleOnTrue(Commands.startEnd(m_ballvision::conePipe, m_ballvision::cubePipe, m_ballvision));

    /* Operator Joystick Inputs */
    // Arm rotation control
    opJoystick.button(2).whileTrue(Commands.startEnd(() -> m_manipulator.lowerArm(0.5),() -> m_manipulator.stopRotation(), m_manipulator));
    opJoystick.button(3).whileTrue(Commands.startEnd(() -> m_manipulator.raiseArm(.5), m_manipulator::stopRotation, m_manipulator));
    // Intake pinch control
    opJoystick.button(1).onTrue(Commands.runOnce(m_manipulator::pinchToggle, m_manipulator));

  }

  // Forwards driver controller to commands for inline use
  public CommandXboxController getXboxController() {
    return drController;
}

  // Forwards operator joystick to commands for inline use
  public CommandJoystick getJoystick() {
    return opJoystick;
  }

  // Autonomous commands 
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public void teleopPeriodic() {
  }
  

}
