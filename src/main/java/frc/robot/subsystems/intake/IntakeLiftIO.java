package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeLiftIO {
    
    @AutoLog
    public static class IntakeLiftIOInputs {
        public double rodillosCorriente = 0.0;
        public double rodillosVoltajeAplicado = 0.0;
        public double rodillosVelocidad = 0.0;

        public double brazoEncoderRaw = 0.0; // 0 a 1 del absoluto
        public double brazoPosicionGrados = 0.0; // Ya convertido
        public double brazoVelocidadGradosPorSeg = 0.0;
        public double brazoCorriente = 0.0;
        public double brazoVoltajeAplicado = 0.0;
    }
    /**
     * Actualiza las entradas del sistema.
     * @param inputs Entradas del sistema.
     */
    public default void updateInputs(IntakeLiftIOInputs inputs) {}

    /**
     * Inyecta un voltaje al motor del brazo.
     * @param volts Volts a inyectar.
     */
    public default void setVoltajeLift(double volts) {}
    
    
    /**
     * Detiene el brazo.
     *
     */
    public default void stopLift() {}

    /**
     * Reinicia el encoder del brazo a la posición inicial. 
     */
    public default void resetEncoder() {}
}