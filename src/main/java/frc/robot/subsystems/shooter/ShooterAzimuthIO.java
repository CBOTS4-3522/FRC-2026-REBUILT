package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterAzimuthIO {
    
    @AutoLog
    public static class ShooterAzimuthIOInputs {
        // --- Azimuth (Torreta - Spark MAX) ---
        public double azimuthPositionDegrees = 0.0;
        public double azimuthVelocityDegPerSec = 0.0; 
        public double azimuthAppliedVolts = 0.0;
        public double azimuthCurrentAmps = 0.0;
        public boolean isAzimuthLimitSwitchPressed = false; 
        
        // --- Pivot (Chamfle - Servo PWM) ---
        public double pivotAngleDegrees = 0.0; 
    }

    public default void updateInputs(ShooterAzimuthIOInputs inputs) {}

    // Azimuth (Torreta)
    public default void setAzimuthVoltage(double volts) {}
    public default void stopAzimuth() {}
    public default void setAzimuthZero() {}

    // Pivot (Chamfle)
    public default void setPivotAngle(double degrees) {}
    

}