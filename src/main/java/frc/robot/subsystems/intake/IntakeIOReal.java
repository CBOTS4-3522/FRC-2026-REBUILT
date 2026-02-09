package frc.robot.subsystems.intake;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class IntakeIOReal implements IntakeIO {

    
    // DEFINICIÓN DE HARDWARE
    private final SparkMax motorRodillos;
    private final SparkMax motorBrazo;
    private final DutyCycleEncoder encoder;

    public IntakeIOReal() {
        
     
        //Configuracion del encoder
        encoder = new DutyCycleEncoder(Constants.Intake.kCanalEncoder);
        encoder.setConnectedFrequencyThreshold(1);

        // DEFINICIÓN DE HARDWARE
        motorRodillos = new SparkMax(Constants.Intake.kMotorID, MotorType.kBrushless);
        motorBrazo = new SparkMax(Constants.Intake.kMotorID2, MotorType.kBrushed);

        // CONFIGURACION SPARKS
        SparkMaxConfig configRodillos = new SparkMaxConfig();
        configRodillos.idleMode(IdleMode.kCoast); // Rodillos libres
        configRodillos.smartCurrentLimit(40);

        SparkMaxConfig configBrazo = new SparkMaxConfig();
        configBrazo.idleMode(IdleMode.kBrake); // Brazo frenado para que aguante
        configBrazo.smartCurrentLimit(40);
        configBrazo.inverted(false); // En caso que este al revez

        // Configurar motores
        motorRodillos.configure(configRodillos, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorBrazo.configure(configBrazo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        
        //ACTUALIZACION DEL INPUTS
        inputs.rodillosCorriente = motorRodillos.getOutputCurrent();
        inputs.rodillosVelocidad = motorRodillos.getEncoder().getVelocity();
        inputs.rodillosVoltajeAplicado = motorRodillos.getBusVoltage() * motorRodillos.getAppliedOutput();

        inputs.brazoEncoderRaw = encoder.get();
        inputs.brazoCorriente = motorBrazo.getOutputCurrent();
        inputs.brazoVoltajeAplicado = motorBrazo.getBusVoltage() * motorBrazo.getAppliedOutput();
    }

    //METODOS PARA MOVER MOTORES
    @Override
    public void setVoltajeRodillos(double volts) {
        motorRodillos.setVoltage(volts);
    }

    @Override
    public void setVoltajeBrazo(double volts) {
        motorBrazo.setVoltage(volts);
    }

    @Override
    public void stopRodillos() {
        motorRodillos.stopMotor();
    }

    @Override
    public void stopBrazo() {
        motorBrazo.stopMotor();
    }



}