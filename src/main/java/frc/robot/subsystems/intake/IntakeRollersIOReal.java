/*
 * IntakeRollersIOReal.java
 *
 * Implementación de hardware para los rodillos usando motores Brushless (NEO).
 * Opera bajo un principio simple de Lazo Abierto (Voltaje Directo).
 */
package frc.robot.subsystems.intake;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class IntakeRollersIOReal implements IntakeRollersIO {
    private final SparkMax motorRodillos;

    public IntakeRollersIOReal() {
        motorRodillos = new SparkMax(Constants.Intake.kMotorRollerID, MotorType.kBrushless);
        
        SparkMaxConfig configRodillos = new SparkMaxConfig();
        // Coast Mode: Permite que los rodillos giren libremente si se corta la energía.
        // Esto evita que una pelota se quede atorada y no pueda sacarse manualmente.
        configRodillos.idleMode(IdleMode.kCoast);
        configRodillos.smartCurrentLimit(35); // Protección del material de los rodillos
                 
        motorRodillos.configure(configRodillos, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        inputs.rodillosCorriente = motorRodillos.getOutputCurrent();
        inputs.rodillosVelocidad = motorRodillos.getEncoder().getVelocity();
        inputs.rodillosVoltajeAplicado = motorRodillos.getBusVoltage() * motorRodillos.getAppliedOutput();
    }

    @Override
    public void setVoltajeRodillos(double volts) {
        motorRodillos.setVoltage(volts);
    }

    @Override
    public void stopRodillos() {
        motorRodillos.stopMotor();
    }
}