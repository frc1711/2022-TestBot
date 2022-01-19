package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;
import frc.team1711.swerve.subsystems.SwerveDrive;
import frc.team1711.swerve.util.Angles;

public class Swerve extends AutoSwerveDrive {
	
	private static final double trackToWheelbaseRatio = 21/18.;
	
	private static final int
		frontLeftSteerID = 6,
		frontRightSteerID = 4,
		rearLeftSteerID = 8,
		rearRightSteerID = 2,
		
		frontLeftDriveID = 5,
		frontRightDriveID = 3,
		rearLeftDriveID = 7,
		rearRightDriveID = 1,
		
		// TODO: Get CANCoder IDs
		frontLeftSteerEncoderID = 0,
		frontRightSteerEncoderID = 0,
		rearLeftSteerEncoderID = 0,
		rearRightSteerEncoderID = 0;
	
	
	private static final double
		driveRelativeSpeed = SwerveDrive.driveRelativeSpeedDefault,
		steerRelativeSpeed = SwerveDrive.steerRelativeSpeedDefault;
	
	private final AHRS gyro;
	
	public Swerve () {
		super(
			new SwerveModule(frontLeftSteerID, frontLeftDriveID, frontLeftSteerEncoderID), // Front left module
			new SwerveModule(frontRightSteerID, frontRightDriveID, frontRightSteerEncoderID), // Front right module
			new SwerveModule(rearLeftSteerID, rearLeftDriveID, rearLeftSteerEncoderID), // Rear left module
			new SwerveModule(rearRightSteerID, rearRightDriveID, rearRightSteerEncoderID), // Rear right module
			trackToWheelbaseRatio);
		setDriveRelativeSpeed(driveRelativeSpeed);
		setSteerRelativeSpeed(steerRelativeSpeed);
		
		gyro = new AHRS();
	}
	
	public void configDirectionEncoders () {
		((SwerveModule)flWheel).configDirectionEncoder();
		((SwerveModule)frWheel).configDirectionEncoder();
		((SwerveModule)rlWheel).configDirectionEncoder();
		((SwerveModule)rrWheel).configDirectionEncoder();
	}
	
	@Override
	public double getGyroAngle () {
		return Angles.wrapDegrees(gyro.getAngle());
	}
	
	@Override
	public void resetGyro () {
		gyro.calibrate();
		gyro.reset();
	}
	
}