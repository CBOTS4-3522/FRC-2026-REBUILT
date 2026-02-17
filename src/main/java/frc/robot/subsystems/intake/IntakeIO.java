package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double rodillosCorriente = 0.0;
        public double rodillosVoltajeAplicado = 0.0;
        public double rodillosVelocidad = 0.0;

        public double brazoEncoderRaw = 0.0; // 0 a 1 del absoluto
        public double brazoPosicionGrados = 0.0; // Ya convertido
        public double brazoCorriente = 0.0;
        public double brazoVoltajeAplicado = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVoltajeRodillos(double volts) {}
    
    public default void stopRodillos() {}

    public default void setVoltajeBrazo(double volts) {}
    
    // --- NUEVO MÉTODO PARA SMART MOTION ---
    /**
     * Mueve el brazo usando Smart Motion en el Spark Max.
     * @param grados Objetivo en grados.
     * @param ffVolts Voltaje arbitrario (Feedforward de gravedad) para ayudar al motor.
     */
    public default void setBrazoPosicion(double grados, double ffVolts) {}

    public default void stopBrazo() {}
}