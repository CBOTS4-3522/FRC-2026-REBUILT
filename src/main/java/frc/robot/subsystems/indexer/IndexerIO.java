package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    public static class IndexerIOInputs {
        // Datos del motor Brushless (Bandas)
        public double bandasVelocidadReal = 0.0;
        public double bandasCorriente = 0.0;
        public double bandasVoltaje = 0.0;

        // Datos del motor Brushed (Mecanum - Redlines no suelen llevar encoder, así que quitamos velocidad)
        public double mecanumCorriente = 0.0;
        public double mecanumVoltaje = 0.0;
        public double mecanumVelocidadReal = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setPorcentajeMotores(double pctBandas, double pctMecanum) {}
    
    public default void stopMotores() {}
}