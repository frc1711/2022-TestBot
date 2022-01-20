package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;

import frc.team1711.swerve.subsystems.AutoSwerveWheel;
import frc.team1711.swerve.util.Angles;

public class SwerveModule extends AutoSwerveWheel {
	
	// The reciprocal of this value is the unit in revolutions
	// with which the direction absolute offset is stored
	private static final int directionOffsetPrecision = 1000;
		
	private static final double
		steerPIDkp = 0.1,
		steerPIDki = 0,
		steerPIDkd = 0;
	
	private final CANCoder steerEncoder;
	private final PIDController steerPID;
	private final double directionAbsoluteOffset;
	private final CANSparkMax driveController, steerController;
	
	public SwerveModule (int steerControllerID, int driveControllerID, int steerEncoderID) {
		driveController = new CANSparkMax(steerControllerID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steerController = new CANSparkMax(driveControllerID, CANSparkMaxLowLevel.MotorType.kBrushless);
		
		driveController.setIdleMode(IdleMode.kBrake);
		steerController.setIdleMode(IdleMode.kBrake);
		
		directionAbsoluteOffset = getDirectionAbsoluteOffset();
		steerEncoder = new CANCoder(steerEncoderID);
		
		steerPID = new PIDController(steerPIDkp, steerPIDki, steerPIDkd);
	}
	
	@Override
	protected double getPositionDifference () {
		return 0; // No autonomous capability
	}
	
	@Override
	protected void resetDriveEncoder () { } // No autonomous capability
	
	// Direction methods
	@Override
	protected double getDirection () {
		return Angles.wrapDegrees(getRawDirection());
	}
	
	private double getRawDirection () {
		return steerEncoder.getAbsolutePosition() - directionAbsoluteOffset;
	}
	
	// Absolute encoder direction offset
	private double getDirectionAbsoluteOffset () {
		try {
			// Retrieve direction from absolute encoder's arbitrary 0 value
			return (double) steerEncoder.configGetCustomParam(0, directionOffsetPrecision) / directionOffsetPrecision;
		} catch (NullPointerException e) { return 0; }
	}
	
	private int createNewDirectionAbsoluteOffset () {
		// Creates a new value for the encoder's direction absolute offset
		// based on the assumption that the wheel is currently facing directly forward
		return (int)Math.round(steerEncoder.getAbsolutePosition() * directionOffsetPrecision);
	}
	
	public void configDirectionEncoder () {
		// Create the cancoder configuration
		CANCoderConfiguration config = new CANCoderConfiguration();
		
		// Use absolute position
		config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		
		// Output value from 0 to 360
		config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
		
		// Clockwise is positive displacement
		config.sensorDirection = true;
		
		// Sets the direction absolute offset
		config.customParam0 = createNewDirectionAbsoluteOffset();
		
		// Flashes the configuration
		steerEncoder.configAllSettings(config, 100);
	}
	
	// Controlling steering
	@Override
	protected void setDirection (double targetDirection) {
		// Gets desired change in direction by wrapping the difference
		// between where we want to be and where we with zero as the center
		double directionChange = Angles.wrapDegreesZeroCenter(targetDirection - getDirection());
		
		// PID loop between 0 (representing current value) and the number
		// of revolutions we want to change the direction by
		double steerSpeed = steerPID.calculate(0, directionChange / 360);
		
		// Sets steering controller
		steerController.set(steerSpeed);
	}
	
	@Override
	protected void stopSteering () {
		steerController.set(0);
	}
	
	@Override
	protected void setDriveSpeed (double speed) {
		driveController.set(speed);
	}
	
}