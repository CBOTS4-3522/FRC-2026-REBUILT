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
        // 1. MOTOR VIEJO (Bandas - NEO Brushless)
        motorBandas = new SparkMax(Constants.Indexer.kMotorBandasID, MotorType.kBrushless);
        SparkMaxConfig configBandas = new SparkMaxConfig();
        configBandas.idleMode(IdleMode.kCoast);
        configBandas.smartCurrentLimit(65);
        configBandas.inverted(false);
        configBandas.openLoopRampRate(0.25);
        motorBandas.configure(configBandas, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // 2. MOTOR NUEVO (Mecanum - NEO Brushless)
        // ¡Súper importante el MotorType.kBrushed para que no tiemble!
        motorMecanum = new SparkMax(Constants.Indexer.kMotorMecanumID, MotorType.kBrushless);
        SparkMaxConfig configMecanum = new SparkMaxConfig();
        configMecanum.idleMode(IdleMode.kCoast);
        configMecanum.smartCurrentLimit(35); // Límite de seguridad para Redlines
        configMecanum.inverted(false); // Cambia a true si gira al revés
        configMecanum.openLoopRampRate(0.25);
        motorMecanum.configure(configMecanum, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
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