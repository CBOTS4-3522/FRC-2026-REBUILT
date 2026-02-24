package frc.robot.subsystems.intake;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class IntakeLiftOIReal implements IntakeLiftIO {

    private final SparkMax motorBrazo;
    private final AbsoluteEncoder encoder;
    private final SparkClosedLoopController controladorBrazo;
    // Convertimos configBrazo a nivel de clase para poder modificarla después
    private final SparkMaxConfig configBrazo = new SparkMaxConfig();

    public IntakeLiftOIReal() {
       

        // CORRECCIÓN 1: Regresado a kBrushed como confirmaste.
        motorBrazo = new SparkMax(Constants.Intake.kMotorLiftID, MotorType.kBrushed);

        encoder = motorBrazo.getAbsoluteEncoder();
        controladorBrazo = motorBrazo.getClosedLoopController();

        SparkMaxConfig configRodillos = new SparkMaxConfig();
        configRodillos.idleMode(IdleMode.kCoast);
        configRodillos.smartCurrentLimit(60);

        configBrazo.idleMode(IdleMode.kBrake);
        configBrazo.smartCurrentLimit(60);
        configBrazo.inverted(false);

        // 1. Configurar Encoder Absoluto (Factor de conversión)
        configBrazo.absoluteEncoder.positionConversionFactor(360.0);
        configBrazo.absoluteEncoder.velocityConversionFactor(360.0 / 60.0);
        configBrazo.absoluteEncoder.inverted(true);

        configBrazo.absoluteEncoder.zeroOffset(0);

        // 2. Configurar PID Interno
        configBrazo.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // ==========================================
        // SLOT 0: SUBIR (Elegante y Controlado)
        // ==========================================
        configBrazo.closedLoop.p(Constants.Intake.kP, ClosedLoopSlot.kSlot0);
        configBrazo.closedLoop.i(0.0, ClosedLoopSlot.kSlot0);
        configBrazo.closedLoop.d(Constants.Intake.kD, ClosedLoopSlot.kSlot0);
        
        // Velocidad moderada (ej. 150) y aceleración suave (ej. 100)
        configBrazo.closedLoop.maxMotion.cruiseVelocity(2000, ClosedLoopSlot.kSlot0); 
        configBrazo.closedLoop.maxMotion.maxAcceleration(2000, ClosedLoopSlot.kSlot0); 
        configBrazo.closedLoop.maxMotion.allowedProfileError(Constants.Intake.kTolerancyDegrees, ClosedLoopSlot.kSlot0);

        // ==========================================
        // SLOT 1: BAJAR (Aventar rápido)
        // ==========================================
        configBrazo.closedLoop.p(Constants.Intake.kP, ClosedLoopSlot.kSlot1); 
        configBrazo.closedLoop.i(0.0, ClosedLoopSlot.kSlot1);
        configBrazo.closedLoop.d(Constants.Intake.kD, ClosedLoopSlot.kSlot1);
        
        // Velocidad máxima alta (ej. 450) y aceleración agresiva (ej. 400)
        configBrazo.closedLoop.maxMotion.cruiseVelocity(5000, ClosedLoopSlot.kSlot1); 
        configBrazo.closedLoop.maxMotion.maxAcceleration(5000, ClosedLoopSlot.kSlot1); 
        configBrazo.closedLoop.maxMotion.allowedProfileError(Constants.Intake.kTolerancyDegrees, ClosedLoopSlot.kSlot1);

        // Aplicar configuraciones al motor (AQUÍ ES DONDE DABA EL CRASH, ya no debería
        // darlo)
        motorBrazo.configure(configBrazo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setPID(double p, double i, double d) {
        // Actualizamos Slot 0 (Subir)
        configBrazo.closedLoop.p(p, ClosedLoopSlot.kSlot0);
        configBrazo.closedLoop.i(i, ClosedLoopSlot.kSlot0);
        configBrazo.closedLoop.d(d, ClosedLoopSlot.kSlot0);

        // Actualizamos Slot 1 (Bajar)
        configBrazo.closedLoop.p(p, ClosedLoopSlot.kSlot1);
        configBrazo.closedLoop.i(i, ClosedLoopSlot.kSlot1);
        configBrazo.closedLoop.d(d, ClosedLoopSlot.kSlot1);

        motorBrazo.configure(configBrazo, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(IntakeLiftIOInputs inputs) {

        // getPosition() ya devuelve GRADOS y ya tiene el offset aplicado correctamente
        inputs.brazoPosicionGrados = encoder.getPosition();
        inputs.brazoEncoderRaw = encoder.getPosition() / 360.0;
        inputs.brazoVelocidadGradosPorSeg = encoder.getVelocity();

        inputs.brazoCorriente = motorBrazo.getOutputCurrent();
        inputs.brazoVoltajeAplicado = motorBrazo.getBusVoltage() * motorBrazo.getAppliedOutput();
    }

    @Override
    public void setBrazoPosicion(double grados, double ffVolts) {
        // Leemos la posición actual para saber si vamos a subir o bajar
        double posicionActual = encoder.getPosition();
        
        // Si el objetivo es MENOR que la posición actual, vamos hacia abajo. 
        // Usamos el Slot 1 (Violento). Si no, usamos el Slot 0 (Elegante).
        ClosedLoopSlot slotAUsar = (grados < posicionActual) ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;

        controladorBrazo.setSetpoint(
            grados, 
            ControlType.kPosition, 
            slotAUsar, // <-- El Spark cambia de perfil instantáneamente
            ffVolts 
        );
    }


    @Override
    public void setVoltajeBrazo(double volts) {
        motorBrazo.setVoltage(volts);
    }

    @Override
    public void stopBrazo() {
        motorBrazo.stopMotor();
    }
}