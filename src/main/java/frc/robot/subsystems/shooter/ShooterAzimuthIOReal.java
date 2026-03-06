package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ShooterAzimuthIOReal implements ShooterAzimuthIO {

    private final SparkMax motorAzimuth;
    private final SparkAbsoluteEncoder azimuthEncoder;

    private final Servo pivotServo; // NUEVO: Servo para el chamfle

    // --- Lógica de Vueltas Infinitas (Rollover) ---
    private double lastRawPosition = 0.0;
    private double azimuthTotalRotations = 0.0;
    private final double kEncoderGearRatio = 6.0;
    private double angleOffset = 0.0;
    SparkMaxConfig configFlywheel = new SparkMaxConfig();

    public ShooterAzimuthIOReal() {

        // --- CONFIGURACIÓN AZIMUTH (TORRETA) ---
        motorAzimuth = new SparkMax(Constants.shooter.azimuth.kID, MotorType.kBrushless);
        // Leemos el encoder desde el puerto del adaptador de REV
        azimuthEncoder = motorAzimuth.getAbsoluteEncoder();

        SparkMaxConfig configAzimuth = new SparkMaxConfig();
        configAzimuth.idleMode(IdleMode.kBrake); // Mejor Brake para la torreta
        configAzimuth.smartCurrentLimit(40); // Bajamos el límite para seguridad
        configAzimuth.inverted(true);
        configAzimuth.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        configAzimuth.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);

        configAzimuth.absoluteEncoder.inverted(true); // (O false)

        motorAzimuth.configure(configAzimuth, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- SERVO PIVOT ---
        // Asumiendo que lo conectas al puerto PWM 0 de la RoboRIO
        pivotServo = new Servo(0);

        // INICIALIZACIÓN DE VUELTAS
        // azimuthEncoder.getPosition() nos da un valor limpio de 0.0 a 1.0 por defecto
        lastRawPosition = azimuthEncoder.getPosition();
    }

    @Override
    public void updateInputs(ShooterAzimuthIOInputs inputs) {
        // 1. Leer valor crudo del adaptador (0.0 a 1.0)
        double currentRaw = azimuthEncoder.getPosition();

        // 2. Detectar vueltas completas del pequeño piñón
        double diff = currentRaw - lastRawPosition;
        if (diff < -0.5) {
            azimuthTotalRotations += 1.0;
        } else if (diff > 0.5) {
            azimuthTotalRotations -= 1.0;
        }
        lastRawPosition = currentRaw;

        // 3. Calcular grados absolutos de la torreta
        double totalEncoderTurns = azimuthTotalRotations + currentRaw;
        double rawDegrees = (totalEncoderTurns / kEncoderGearRatio) * 360.0;

        inputs.azimuthPositionDegrees = rawDegrees - angleOffset;
        inputs.isAzimuthLimitSwitchPressed = motorAzimuth.getReverseLimitSwitch().isPressed();

        // --- Resto de inputs ---
        inputs.azimuthAppliedVolts = motorAzimuth.getBusVoltage() * motorAzimuth.getAppliedOutput();
        inputs.azimuthCurrentAmps = motorAzimuth.getOutputCurrent();

        inputs.pivotAngleDegrees = pivotServo.getAngle(); // Para el log

    }

    @Override
    public void setAzimuthZero() {
        double currentRawDegrees = ((azimuthTotalRotations + lastRawPosition) / kEncoderGearRatio) * 360.0;
        this.angleOffset = currentRawDegrees;
    }

    @Override
    public void setAzimuthVoltage(double volts) {
        motorAzimuth.setVoltage(volts);
    }

    @Override
    public void setPivotAngle(double degrees) {
        // Los servos estándar de FRC aceptan de 0 a 180 grados
        pivotServo.setAngle(degrees);
    }

    @Override
    public void setAzimuthPosition(double degrees){
        double currentRawDegrees = ((azimuthTotalRotations + lastRawPosition) / kEncoderGearRatio) * 360.0;
        this.angleOffset = currentRawDegrees - degrees;
    }

    @Override
    public void stopAzimuth() {
        motorAzimuth.stopMotor();
    }

}