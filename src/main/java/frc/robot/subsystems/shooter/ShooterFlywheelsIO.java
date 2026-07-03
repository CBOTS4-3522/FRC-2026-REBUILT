/*
 * ShooterFlywheelsIO.java
 *
 * Interfaz para los motores propulsores (Flywheels).
 */
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelsIO {
    
    @AutoLog
    public static class ShooterFlywheelsIOInputs {
        public double flywheelVelocityRPMLider = 0.0;
        public double flywheelVelocityRPMFollower = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        public double flywheelTemplider = 0.0;
        public double flywheeltempFollower = 0.0;
        
        public boolean errorEncoderLider = false;
        public boolean errorEncoderFollower = false;
    }
    /**
     * Relaciona las entradas con las entradas reales para que sean registradas
     * @param inputs las entradas para ser registradas
     */
    public default void updateInputs(ShooterFlywheelsIOInputs inputs) {}
    /**
     * Inyecta un voltage directo al motor que lanza las pelotas
     * @param volts voltaje a mandar (De 0 a 12)
     */
    public default void setFlywheelVoltage(double volts) {}
    /**
     * Utiliza un control preciso para enviar una cantidad de revoluciones especificas al motor de lanzamiento
     * @param rpm revoluciones para enviar
     * @param ffVolts cantidad de voltaje necesario para empezar a girar
     */
    public default void setFlywheelVelocity(double rpm, double ffVolts) {}
    public default void stopFlywheel() {}
    public default void setPID(double p, double i, double d) {}
}