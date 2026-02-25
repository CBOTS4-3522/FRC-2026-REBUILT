package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {
        // --- Flywheels ---
        public double flywheelVelocityRPMLider = 0.0;
        public double flywheelVelocityRPMFollower = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        public double flywheelTemplider = 0.0;
        public double flywheeltempFollower = 0.0;
        
        // --- Azimuth (Torreta - Spark MAX) ---
        public double azimuthPositionDegrees = 0.0;
        public double azimuthVelocityDegPerSec = 0.0; 
        public double azimuthAppliedVolts = 0.0;
        public double azimuthCurrentAmps = 0.0;
        public boolean isAzimuthLimitSwitchPressed = false; 
        
        // --- Pivot (Chamfle - Servo PWM) ---
        public double pivotAngleDegrees = 0.0; 
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    // Flywheel
    public default void setFlywheelVoltage(double volts) {}
    public default void setFlywheelVelocity(double rpm, double ffVolts) {}
    public default void stopFlywheel() {}

    // Azimuth (Torreta)
    public default void setAzimuthVoltage(double volts) {}
    public default void stopAzimuth() {}
    public default void setAzimuthZero() {}

    // Pivot (Chamfle)
    public default void setPivotAngle(double degrees) {}
    
    //PID flywheels
    public default void setPID(double p, double i, double d) {}
}