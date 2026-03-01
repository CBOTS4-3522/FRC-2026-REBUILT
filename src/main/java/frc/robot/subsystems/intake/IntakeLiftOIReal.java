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

        
        motorBrazo = new SparkMax(Constants.Intake.kMotorLiftID, MotorType.kBrushed);

        configBrazo.idleMode(IdleMode.kBrake);
        configBrazo.smartCurrentLimit(40);
        configBrazo.inverted(false);
        
        encoderRelativo = motorBrazo.getEncoder();
        // Aplicar configuraciones al motor (AQUÍ ES DONDE DABA EL CRASH, ya no debería
        // darlo)
        motorBrazo.configure(configBrazo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void resetEncoder() {
        // Le decimos al motor: "Donde estás ahora mismo, es el cero absoluto"
        encoderRelativo.setPosition(0.0); 
    }

    @Override
    public void updateInputs(IntakeLiftIOInputs inputs) {

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