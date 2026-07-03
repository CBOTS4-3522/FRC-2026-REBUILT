/*
 * IntakeLiftOIReal.java
 *
 * Implementación física (Hardware-in-the-Loop) para el brazo del Intake.
 * Enlaza la lógica de control con el controlador SparkMax y un motor con escobillas (Brushed).
 */
package frc.robot.subsystems.intake;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

public class IntakeLiftOIReal implements IntakeLiftIO {
    private final SparkMax motorBrazo;
    private final SparkMaxConfig configBrazo = new SparkMaxConfig();
    private final RelativeEncoder encoderRelativo;

    public IntakeLiftOIReal() {
        // Inicialización del motor en modo Brushed (Escobillas)
        motorBrazo = new SparkMax(Constants.Intake.kMotorLiftID, MotorType.kBrushed);
        
        configBrazo.idleMode(IdleMode.kBrake); // Freno activo para evitar que la gravedad lo tumbe
        configBrazo.smartCurrentLimit(40); // Límite de seguridad en el bus CAN
        configBrazo.inverted(false);
                 
        encoderRelativo = motorBrazo.getEncoder();
               
        // Aplicación de configuración persistente
        motorBrazo.configure(configBrazo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void resetEncoder() {
        // Establece el "Datum" o punto de referencia cero en la posición actual
        encoderRelativo.setPosition(0.0); 
    }

    @Override
    public void updateInputs(IntakeLiftIOInputs inputs) {
        // Monitoreo de telemetría de hardware (Esencial para la lógica de Stall Detection)
        inputs.brazoCorriente = motorBrazo.getOutputCurrent();
        inputs.brazoVoltajeAplicado = motorBrazo.getBusVoltage() * motorBrazo.getAppliedOutput();
    }

    @Override
    public void setVoltajeLift(double volts) {
        motorBrazo.setVoltage(volts);
    }

    @Override
    public void stopLift() {
        motorBrazo.stopMotor();
    }
}