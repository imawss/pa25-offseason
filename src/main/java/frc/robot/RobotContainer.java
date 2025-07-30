// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.susbsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.susbsystems.drivetrain.enums.DriveType;
import frc.robot.susbsystems.drivetrain.implementations.SimGyro;
import frc.robot.susbsystems.drivetrain.implementations.SimModule;
import frc.robot.utils.Logger;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(ControllerConstants.CONTROLLER_PORT);

  Trigger x = controller.x();
  Trigger a = controller.a();
  Trigger b = controller.b();
  Trigger y = controller.y();
  Trigger rb = controller.rightBumper();
  Trigger rt = controller.rightTrigger(0.1);
  Trigger lb = controller.leftBumper();
  Trigger lt = controller.leftTrigger(0.1);

  Trigger povUp = controller.povUp();
  Trigger povDown = controller.povDown();
  Trigger povLeft = controller.povLeft();
  Trigger povRight = controller.povRight();

  SwerveDriveSimulation drivetrainSimulation;

  DrivetrainSubsystem drivetrainSubsystem;

  DriveCommand fieldRelativeDriveCommand;
  DriveCommand robotRelativeDriveCommand;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {

    drivetrainSimulation = new SwerveDriveSimulation(DrivetrainConstants.DRIVETRAIN_CONFIG,
        RobotConstants.INITIAL_POSE);

    SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSimulation);
    SimulatedArena.getInstance().resetFieldForAuto();

    drivetrainSubsystem = new DrivetrainSubsystem(
        new SimGyro(drivetrainSimulation.getGyroSimulation()),
        new SimModule(drivetrainSimulation.getModules()[0]),
        new SimModule(drivetrainSimulation.getModules()[1]),
        new SimModule(drivetrainSimulation.getModules()[2]),
        new SimModule(drivetrainSimulation.getModules()[3]));

    autoChooser = AutoBuilder.buildAutoChooser();

    this.robotRelativeDriveCommand = new DriveCommand(
        drivetrainSubsystem,
        DriveType.RobotRelative,
        () -> Math.abs(controller.getLeftX()) > ControllerConstants.DEADBAND ? controller.getLeftX() * -5 : 0,
        () -> Math.abs(controller.getLeftY()) > ControllerConstants.DEADBAND ? controller.getLeftY() * -5 : 0,
        () -> Math.abs(controller.getRightX()) > ControllerConstants.DEADBAND ? controller.getRightX() * -5 : 0);

    this.fieldRelativeDriveCommand = new DriveCommand(
        drivetrainSubsystem,
        DriveType.FieldRelative,
        () -> Math.abs(controller.getLeftX()) > ControllerConstants.DEADBAND ? controller.getLeftX() * -5 : 0,
        () -> Math.abs(controller.getLeftY()) > ControllerConstants.DEADBAND ? controller.getLeftY() * -5 : 0,
        () -> Math.abs(controller.getRightX()) > ControllerConstants.DEADBAND ? controller.getRightX() * -5 : 0);

    configureBindings();
  }

  public void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(fieldRelativeDriveCommand);

    x.onTrue(Commands.runOnce(() -> drivetrainSubsystem.setPose(drivetrainSimulation.getSimulatedDriveTrainPose())));
  }

  public Command getAutonomousCommand() {

    return AutoBuilder.buildAuto(autoChooser.getSelected().getName());
  }
  
  public void autonomousPeriodic() {
    drivetrainSubsystem.setPose(drivetrainSimulation.getSimulatedDriveTrainPose()); //update odometry for path planning (this will not be used in reality)
  }

  public void periodic() {

    if (AutoBuilder.isConfigured()) {
      Logger.log("AutoChooser/SelectedAuto", autoChooser);
    }

    Logger.log("Drivetrain/Poses/SimPose", drivetrainSimulation.getSimulatedDriveTrainPose());
  }
}