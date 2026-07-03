/*
 * IndexerIOReal.java
 *
 * Implementación de hardware para el Indexer.
 * Gestiona la inicialización de los controladores SparkMax en el bus CAN.
 */
package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {
    private final SparkMax motorBandas;
    private final SparkMax motorMecanum;

    public IndexerIOReal() {
        // 1. MOTOR BANDAS (NEO Brushless)
        motorBandas = new SparkMax(Constants.Indexer.kMotorBandasID, MotorType.kBrushless);
        SparkMaxConfig configBandas = new SparkMaxConfig();
        
        // kCoast: Esencial en mecanismos de ingesta para que las pelotas no se atoren 
        // si el robot pierde energía repentinamente.
        configBandas.idleMode(IdleMode.kCoast);
        configBandas.smartCurrentLimit(65);
        configBandas.inverted(false);
        configBandas.openLoopRampRate(0.25); // Rampa de aceleración para evitar picos de corriente
        
        motorBandas.configure(configBandas, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // 2. MOTOR MECANUM (Redline/Brushed modificado)
        // Nota técnica: Asegurar que el MotorType coincida con el hardware real (Brushed vs Brushless)
        // para evitar comportamientos erráticos (Jittering).
        motorMecanum = new SparkMax(Constants.Indexer.kMotorMecanumID, MotorType.kBrushless);
        SparkMaxConfig configMecanum = new SparkMaxConfig();
        configMecanum.idleMode(IdleMode.kCoast);
        configMecanum.smartCurrentLimit(35);
        configMecanum.inverted(false);
        configMecanum.openLoopRampRate(0.25);
        
        motorMecanum.configure(configMecanum, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Recolección de telemetría de los controladores
        inputs.bandasVelocidadReal = motorBandas.getEncoder().getVelocity();
        inputs.bandasCorriente = motorBandas.getOutputCurrent();
        inputs.bandasVoltaje = motorBandas.getBusVoltage() * motorBandas.getAppliedOutput();
        
        inputs.mecanumCorriente = motorMecanum.getOutputCurrent();
        inputs.mecanumVoltaje = motorMecanum.getBusVoltage() * motorMecanum.getAppliedOutput();
        inputs.mecanumVelocidadReal = motorMecanum.getEncoder().getVelocity();
    }

    @Override
    public void setPorcentajeMotores(double pctBandas, double pctMecanum) {
        motorBandas.set(pctBandas);
        motorMecanum.set(pctMecanum);
    }

    @Override
    public void stopMotores() {
        motorBandas.stopMotor();
        motorMecanum.stopMotor();
    }
}