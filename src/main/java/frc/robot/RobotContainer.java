// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	
	private final Swerve swerveDrive;
	private final Joystick driveController;
	private final SwerveTeleop swerveTeleop;
	
	public RobotContainer () {
		driveController = new Joystick(0);
		
		swerveDrive = new Swerve();
		swerveTeleop = new SwerveTeleop(
			swerveDrive, // TODO: Get axis IDs for joystick
			() -> driveController.getRawAxis(0),	// strafeX
			() -> driveController.getRawAxis(0),	// strafeY
			() -> driveController.getRawAxis(0));	// steering
		
		swerveDrive.setDefaultCommand(swerveTeleop);
	}
	
	public Command getAutonomousCommand () {
		return null;
	}
	
}