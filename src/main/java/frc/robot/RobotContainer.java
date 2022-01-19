// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
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
	}
	
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand () {
		return null;
	}
	
}