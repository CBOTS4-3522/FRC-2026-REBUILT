//aqui le pasamos al codigo que motor y como ocuparlo asi si cambiamos un motor por otro de golpe solamente modificamos aca

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
    //esto define el motor
    private final SparkMax motorAzimuth;
    // y esto el encoder el sensor que registrara las vueltas
    private final SparkAbsoluteEncoder azimuthEncoder;
    //esto define un servomotor
    private final Servo pivotServo; //Servo para el chamfle

    // --- Lógica de Vueltas Infinitas (Rollover) ---
    private double lastRawPosition = 0.0;
    private double azimuthTotalRotations = 0.0;
    private final double kEncoderGearRatio = 6.0;
    private double angleOffset = 0.0;
    SparkMaxConfig configFlywheel = new SparkMaxConfig();

    public ShooterAzimuthIOReal() {

        // --- CONFIGURACIÓN AZIMUTH (TORRETA) ---
        //aqui le decimos al motor en que ID esta viviendo su controlador y de que tipo es con escobillas o si escobillas
        motorAzimuth = new SparkMax(Constants.shooter.azimuth.kID, MotorType.kBrushless);
        // Leemos el encoder desde el puerto del adaptador de REV
        azimuthEncoder = motorAzimuth.getAbsoluteEncoder();
        
        //se crea la configuracion de este motor
        SparkMaxConfig configAzimuth = new SparkMaxConfig();
        //esto de aqui define el modo del motor, existen 2 brake y coast, el brake frena el motor de golpe cuando lo dejas de ocupar
        //y genera una friccion para que no gire tan facil, luego el coast este lo deja seguir para que frene solo por friccion
        configAzimuth.idleMode(IdleMode.kBrake);
        //el limite de amperaje que le vamos a proporcionar al motor
        configAzimuth.smartCurrentLimit(40); 
        //para que gire para un lado o para otro
        configAzimuth.inverted(true);
        //los estados de los limit switch unos switch que frenan el motor cuando se accionen para que no se decapite solo
        configAzimuth.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        configAzimuth.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        //aqui si el encoder cuenta positivo o cuenta en negativo
        configAzimuth.absoluteEncoder.inverted(true);

        //y aqui le pasamos toda esa configuracion al motor 
        motorAzimuth.configure(configAzimuth, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- SERVO PIVOT ---
        //el pin al que se va a conectar en la rio
        pivotServo = new Servo(0);

        // INICIALIZACIÓN DE VUELTAS
        // da un valor de 0.0 a 1.0
        lastRawPosition = azimuthEncoder.getPosition();
    }

    @Override
    public void updateInputs(ShooterAzimuthIOInputs inputs) {
        // 1. Leer valor crudo del adaptador (0.0 a 1.0)
        double currentRaw = azimuthEncoder.getPosition();

        boolean fallaFisica = motorAzimuth.getFaults().sensor;
        
        // 3. Revisas si hubo un error al intentar comunicarse con el encoder
        // (com.revrobotics.REVLibError.kOk significa que todo está bien)
        boolean fallaComunicacion = (motorAzimuth.getLastError() != com.revrobotics.REVLibError.kOk);

        // Si cualquiera de las dos cosas falla, activamos la alerta
        inputs.fallaEncoderAbsoluto = fallaFisica || fallaComunicacion;

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
    //y aqui le traducimos a la interfaz como es con los motores
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