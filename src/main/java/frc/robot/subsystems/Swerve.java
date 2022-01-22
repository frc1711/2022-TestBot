package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		
		frontLeftSteerEncoderID = 9,
		frontRightSteerEncoderID = 10,
		rearLeftSteerEncoderID = 11,
		rearRightSteerEncoderID = 12;
	
	
	private static final double
		driveRelativeSpeed = SwerveDrive.driveRelativeSpeedDefault,
		steerRelativeSpeed = SwerveDrive.steerRelativeSpeedDefault;
	
	private final AHRS gyro;
	
	public Swerve () {
		super(
			new SwerveModule("FrontLeft", frontLeftSteerID, frontLeftDriveID, frontLeftSteerEncoderID), // Front left module
			new SwerveModule("FrontRight", frontRightSteerID, frontRightDriveID, frontRightSteerEncoderID), // Front right module
			new SwerveModule("RearLeft", rearLeftSteerID, rearLeftDriveID, rearLeftSteerEncoderID), // Rear left module
			new SwerveModule("RearRight", rearRightSteerID, rearRightDriveID, rearRightSteerEncoderID), // Rear right module
			trackToWheelbaseRatio);
		setDriveRelativeSpeed(driveRelativeSpeed);
		setSteerRelativeSpeed(steerRelativeSpeed);
		
		gyro = new AHRS();
	}
	
	public void displayOrientation () {
		SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
		SmartDashboard.putNumber("Front Left Direction", ((SwerveModule)flWheel).getDirection() % 180);
		SmartDashboard.putNumber("Front Right Direction", ((SwerveModule)frWheel).getDirection() % 180);
		SmartDashboard.putNumber("Rear Left Direction", ((SwerveModule)rlWheel).getDirection() % 180);
		SmartDashboard.putNumber("Rear Right Direction", ((SwerveModule)rrWheel).getDirection() % 180);
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