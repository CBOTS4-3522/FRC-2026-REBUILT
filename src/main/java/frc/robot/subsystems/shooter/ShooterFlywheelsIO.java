package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelsIO {
    
    @AutoLog
    public static class ShooterFlywheelsIOInputs {
        // --- Flywheels ---
        public double flywheelVelocityRPMLider = 0.0;
        public double flywheelVelocityRPMFollower = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        public double flywheelTemplider = 0.0;
        public double flywheeltempFollower = 0.0;
        
      
    }

    public default void updateInputs(ShooterFlywheelsIOInputs inputs) {}

    // Flywheel
    public default void setFlywheelVoltage(double volts) {}
    public default void setFlywheelVelocity(double rpm, double ffVolts) {}
    public default void stopFlywheel() {}
    
    //PID flywheels
    public default void setPID(double p, double i, double d) {}
}