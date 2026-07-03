/*
 * IntakeLiftIO.java
 *
 * Interfaz de abstracción de hardware (HAL) para el mecanismo de elevación del Intake.
 * Define la estructura de datos (Inputs) que el motor y sus sensores enviarán 
 * a la capa lógica para su procesamiento y registro (Logging).
 */
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeLiftIO {
    
    @AutoLog
    public static class IntakeLiftIOInputs {
        public double rodillosCorriente = 0.0;
        public double rodillosVoltajeAplicado = 0.0;
        public double rodillosVelocidad = 0.0;
        
        // Datos específicos del Brazo
        public double brazoEncoderRaw = 0.0; // Valor crudo del encoder (0.0 a 1.0)
        public double brazoPosicionGrados = 0.0; 
        public double brazoVelocidadGradosPorSeg = 0.0;
        public double brazoCorriente = 0.0; // CRÍTICO: Usado para la detección de colisiones (Stall)
        public double brazoVoltajeAplicado = 0.0;
    }

    /** Sincroniza el estado del hardware con la estructura de datos lógica. */
    public default void updateInputs(IntakeLiftIOInputs inputs) {}
    
    /** Inyecta un voltaje en lazo abierto (Open-Loop) al motor del brazo. */
    public default void setVoltajeLift(double volts) {}
              
    /** Corta la energía del motor (0 Volts). */
    public default void stopLift() {}
    
    /** Define la posición física actual como el "Cero" lógico del encoder. */
    public default void resetEncoder() {}
}