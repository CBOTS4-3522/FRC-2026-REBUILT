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

public class IntakeIOReal implements IntakeIO {

    private final SparkMax motorRodillos;
    private final SparkMax motorBrazo;
    private final AbsoluteEncoder encoder;
    private final SparkClosedLoopController controladorBrazo; // Controlador interno

    public IntakeIOReal() {
        // ... (Inicialización de motores igual que antes) ...
        motorRodillos = new SparkMax(Constants.Intake.kMotorID, MotorType.kBrushless);
        motorBrazo = new SparkMax(Constants.Intake.kMotorID2, MotorType.kBrushed); // OJO: ¿Seguro es Brushed (con carbones)? Si es NEO/Vortex pon Brushless.
        
        encoder = motorBrazo.getAbsoluteEncoder();
        controladorBrazo = motorBrazo.getClosedLoopController();

        // ... (Config Rodillos igual) ...
        SparkMaxConfig configRodillos = new SparkMaxConfig();
        configRodillos.idleMode(IdleMode.kCoast);
        configRodillos.smartCurrentLimit(40);
        
        // --- CONFIGURACIÓN BRAZO (SMART MOTION) ---
        SparkMaxConfig configBrazo = new SparkMaxConfig();
        configBrazo.idleMode(IdleMode.kBrake);
        configBrazo.smartCurrentLimit(40);
        configBrazo.inverted(false);

        // 1. Configurar Encoder Absoluto (Factor de conversión)
        // Hacemos que el Spark trabaje DIRECTO EN GRADOS.
        // Factor = 360.0. Así getPosition() da grados (0-360).
        configBrazo.absoluteEncoder.positionConversionFactor(360.0);
        configBrazo.absoluteEncoder.velocityConversionFactor(360.0 / 60.0); // deg/sec
        configBrazo.absoluteEncoder.inverted(true); // Ajustar según tu mecánica (1 - raw)
        // configBrazo.absoluteEncoder.zeroOffset(Constants.Intake.kEncoderOffset); // Si quieres el offset en hardware

        // 2. Configurar PID Interno
        // Usamos el encoder absoluto como feedback
        configBrazo.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        
        // PID (Ajustar estos valores en Constants o Elastic)
        configBrazo.closedLoop.p(Constants.Intake.kP); // Empieza bajito, ej. 0.01
        configBrazo.closedLoop.i(0.0);
        configBrazo.closedLoop.d(Constants.Intake.kD);
        
        // 3. Configurar SMART MOTION (El perfil trapezoidal)
        // Slot 0 (puedes usar otro si quieres)
        configBrazo.closedLoop.maxMotion.cruiseVelocity(500); // Grados/segundo
        configBrazo.closedLoop.maxMotion.maxAcceleration(500); // Grados/segundo^2
        configBrazo.closedLoop.maxMotion.allowedProfileError(Constants.Intake.kTolerancyDegrees);

        // Aplicar
        motorRodillos.configure(configRodillos, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorBrazo.configure(configBrazo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rodillosCorriente = motorRodillos.getOutputCurrent();
        inputs.rodillosVelocidad = motorRodillos.getEncoder().getVelocity();
        inputs.rodillosVoltajeAplicado = motorRodillos.getBusVoltage() * motorRodillos.getAppliedOutput();

        // Ahora getPosition() ya devuelve GRADOS (0-360)
        inputs.brazoPosicionGrados = encoder.getPosition(); 
        inputs.brazoEncoderRaw = inputs.brazoPosicionGrados / 360.0; // Por si quieres ver el raw 0-1
        
        inputs.brazoCorriente = motorBrazo.getOutputCurrent();
        inputs.brazoVoltajeAplicado = motorBrazo.getBusVoltage() * motorBrazo.getAppliedOutput();
    }

    @Override
    public void setBrazoPosicion(double grados, double ffVolts) {
        // AQUÍ OCURRE LA MAGIA
        // ControlType.kMAXMotionPositionControl es "Smart Motion"
        // Le mandamos el objetivo (grados) y el feedforward (volts)
        controladorBrazo.setSetpoint(
            grados, 
            ControlType.kMAXMotionPositionControl, 
            ClosedLoopSlot.kSlot0, 
            ffVolts // El Spark suma esto al resultado del PID
        );
    }
    
    // ... otros métodos (setVoltaje, stop) ...
    @Override public void setVoltajeRodillos(double volts) { motorRodillos.setVoltage(volts); }
    @Override public void stopRodillos() { motorRodillos.stopMotor(); }
    @Override public void setVoltajeBrazo(double volts) { motorBrazo.setVoltage(volts); }
    @Override public void stopBrazo() { motorBrazo.stopMotor(); }
}