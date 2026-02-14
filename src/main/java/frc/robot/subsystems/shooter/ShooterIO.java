package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {
        // --- Flywheels ---
        public double flywheelVelocityRPM = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        
        // --- Pivot (Conectado al RIO) ---
        public double azimuthPositionDegrees = 0.0;
        public double pivotVelocityDegPerSec = 0.0; // Velocidad en grados/seg
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        
        // --- Seguridad ---
        public boolean isLimitSwitchPressed = false; // Limit switch DIO 1
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    // Flywheel methods
    public default void setFlywheelVoltage(double volts) {}
    public default void setFlywheelVelocity(double rpm) {}
    public default void stopFlywheel() {}

    // Pivot methods
    public default void setPivotVoltage(double volts) {}
    public default void stopAzimuth() {}
    public default void setPivotZero() {}

}