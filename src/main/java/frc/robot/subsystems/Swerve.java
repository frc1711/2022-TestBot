package frc.robot.subsystems;

import frc.team1711.swerve.subsystems.AutoSwerveDrive;

public class Swerve extends AutoSwerveDrive {

    public Swerve (
        SwerveModule frontLeft,
        SwerveModule frontRight,
        SwerveModule rearLeft,
        SwerveModule rearRight) {
        super(frontLeft, frontRight, rearLeft, rearRight);
    }

    @Override
    public double getGyroAngle() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void resetGyro() {
        // TODO Auto-generated method stub
        
    }

}