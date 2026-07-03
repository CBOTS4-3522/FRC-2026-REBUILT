/*
 * IndexerIO.java
 *
 * Interfaz HAL (Hardware Abstraction Layer) para el sistema de alimentación de proyectiles.
 * Define el contrato de datos (Inputs) que el motor reportará al Logger.
 */
package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    public static class IndexerIOInputs {
        // Datos del motor principal (Bandas)
        public double bandasVelocidadReal = 0.0;
        public double bandasCorriente = 0.0;
        public double bandasVoltaje = 0.0;
        
        // Datos del motor secundario (Mecanum)
        public double mecanumCorriente = 0.0;
        public double mecanumVoltaje = 0.0;
        public double mecanumVelocidadReal = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setPorcentajeMotores(double pctBandas, double pctMecanum) {}
    public default void stopMotores() {}
}