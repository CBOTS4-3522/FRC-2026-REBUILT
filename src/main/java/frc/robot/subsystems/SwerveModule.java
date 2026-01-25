package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    public Rotation2d getCanCoder();

    public SwerveModuleState getState();

    public SwerveModulePosition getPosition();

    public SwerveModuleState getDesiredState();

    public int getModuleNumber();

    public void setModuleNumber(int moduleNumber);

    public void synchronizeEncoders();
    
    public double getOmega();

    public Rotation2d getAngle(); 

    public void setDriveVoltage(double volts);
    
    public double getDriveVoltage();
    
    public void lockAngle();
}