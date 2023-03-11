// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autonomous.scoreDoubleReverse;
import frc.robot.commands.autonomous.scoreReverse;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.DriveForTime;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;

public class RobotContainer {
  // Initializes controllers and subsystems for command mapping uses
  public static CommandXboxController drController = new CommandXboxController(OIConstants.drController);
  public static CommandXboxController opController = new CommandXboxController(OIConstants.opController);
  public static DriveTrain m_drivetrain = new DriveTrain();
  public static Manipulator m_manipulator = new Manipulator();
  // Initializes autonomous chooser for Smart Dashboard
  private SendableChooser<Command> autonChooser = new SendableChooser<>();
  
  // Creates all the autonomous commands
  private final scoreDoubleReverse m_ScoreDoubleReverse = new scoreDoubleReverse(
    m_manipulator, m_drivetrain, drController, .7,4.7, -.3, 1, 2.9);
  private final DriveForTime m_Reverse = new DriveForTime(
    m_drivetrain, .5, 3, drController);
  private final scoreReverse m_ScoreReverseCS = new scoreReverse(
    m_drivetrain, m_manipulator, drController, .7, .3, 3);
  private final scoreReverse m_ScoreReverse = new scoreReverse(
    m_drivetrain, m_manipulator, drController, .7, .3, 4);

  public RobotContainer() {
    // Puts the autonomous chooser on the dashboard
  SmartDashboard.putData("Autonomous Chooser", autonChooser);
  
  // Autonomous chooser for Smart Dashboard
  autonChooser.setDefaultOption("scoreDoubleReverse", m_ScoreDoubleReverse);
  autonChooser.addOption("scoreReverseCS", m_ScoreReverseCS);
  autonChooser.addOption("scoreReverse", m_ScoreReverse);
  autonChooser.addOption("Reverse", m_Reverse);


  CameraServer.startAutomaticCapture().setResolution(320, 280);

    configureBindings();
  }

  private void configureBindings() {

    /* Drivetrain Command to drive the robot */
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain, () -> drController.getLeftY(), () -> drController.getRightX(), drController));

    /* Driver Controller Inputs */
    // Coast/Brake Control
    drController.start().toggleOnTrue(Commands.startEnd(m_drivetrain::setIdleBrake, m_drivetrain::setIdleCoast));

    /* Operator Joystick Inputs */
    // Arm rotation control
    opController.a().whileTrue(Commands.startEnd(() -> m_manipulator.extendArm(1),() -> m_manipulator.stopExtender(), m_manipulator));
    opController.y().whileTrue(Commands.startEnd(() -> m_manipulator.retractArm(.7),() -> m_manipulator.stopExtender(), m_manipulator));
    // Intake pinch control
    opController.rightTrigger().toggleOnTrue(Commands.startEnd(m_manipulator::pinchTrue, m_manipulator::pinchFalse, m_manipulator));
    // WIP Lift setpoint subsystem command mapping
    opController.povUp().whileTrue(Commands.startEnd(() -> m_manipulator.raiseArm(.5), m_manipulator::stopLift, m_manipulator));
    opController.povDown().whileTrue(Commands.startEnd(() -> m_manipulator.lowerArm(.5), m_manipulator::stopLift, m_manipulator));
    
  }

  // Autonomous commands 
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }

  public void robotInit() {
  
  }
  public void teleopInit() {
    m_drivetrain.zeroGyro();
  }
  public void autonomousInit() {
    m_drivetrain.zeroGyro();
    m_manipulator.pinchTrue();
  }
  public void teleopDisabled() {
    m_manipulator.pinchTrue();
  }
}
