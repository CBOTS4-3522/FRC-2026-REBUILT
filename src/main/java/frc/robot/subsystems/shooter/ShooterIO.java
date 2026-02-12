package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    }

    /** Actualiza las variables de inputs con los datos de los sensores */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Control por voltaje (Open Loop) */
    public default void setVoltage(double volts) {}

    /** Control por velocidad (Closed Loop / PID) */
    public default void setVelocity(double rpm) {}

    /** Detener motores */
    public default void stop() {}
    
    /** Configurar PID dinámicamente (opcional, pero útil) */
    public default void configurePID(double kP, double kI, double kD) {}
}