package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {
    
    // --- Motores y Sensores ---
    private final SparkMax motorLider;
    private final SparkMax motorSeguidor;
    private final SparkClosedLoopController controladorFlywheel;
    
    private final SparkMax motorAzimuth;
    private final SparkAbsoluteEncoder azimuthEncoder;
    private final DigitalInput azimuthLimitSwitch; 
    
    private final Servo pivotServo; // NUEVO: Servo para el chamfle
    
    // --- Lógica de Vueltas Infinitas (Rollover) ---
    private double lastRawPosition = 0.0; 
    private double azimuthTotalRotations = 0.0; 
    private final double kEncoderGearRatio = 6.0; 
    private double angleOffset = 0.0;
    SparkMaxConfig configFlywheel = new SparkMaxConfig();

    public ShooterIOSparkMax() {
        // --- CONFIGURACIÓN FLYWHEELS ---
        motorLider = new SparkMax(Constants.shooter.flywheels.kLiderID, MotorType.kBrushless);
        motorSeguidor = new SparkMax(Constants.shooter.flywheels.kSeguidorID, MotorType.kBrushless);
        controladorFlywheel = motorLider.getClosedLoopController();

        
        configFlywheel.idleMode(IdleMode.kCoast);
        configFlywheel.inverted(true); 
        configFlywheel.closedLoopRampRate(0.5);
        configFlywheel.closedLoop.p(Constants.shooter.flywheels.kP);
        configFlywheel.closedLoop.i(Constants.shooter.flywheels.kI);
        configFlywheel.closedLoop.d(Constants.shooter.flywheels.kD);
    

        
        SparkMaxConfig configSeguidor = new SparkMaxConfig();
        configSeguidor.idleMode(IdleMode.kCoast);
        configSeguidor.follow(motorLider, true);

        motorLider.configure(configFlywheel, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorSeguidor.configure(configSeguidor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // --- CONFIGURACIÓN AZIMUTH (TORRETA) ---
        motorAzimuth = new SparkMax(Constants.shooter.azimuth.kID, MotorType.kBrushless);
        // Leemos el encoder desde el puerto del adaptador de REV
        azimuthEncoder = motorAzimuth.getAbsoluteEncoder(); 
        
        SparkMaxConfig configAzimuth = new SparkMaxConfig();
        configAzimuth.idleMode(IdleMode.kBrake); // Mejor Brake para la torreta
        configAzimuth.smartCurrentLimit(40);     // Bajamos el límite para seguridad
        configAzimuth.inverted(false);           
        
        motorAzimuth.configure(configAzimuth, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- SENSORES RIO ---
        azimuthLimitSwitch = new DigitalInput(1);
        
        // --- SERVO PIVOT ---
        // Asumiendo que lo conectas al puerto PWM 0 de la RoboRIO
        pivotServo = new Servo(0); 

        // INICIALIZACIÓN DE VUELTAS
        // azimuthEncoder.getPosition() nos da un valor limpio de 0.0 a 1.0 por defecto
        lastRawPosition = azimuthEncoder.getPosition();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
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

        // --- Resto de inputs ---
        inputs.azimuthAppliedVolts = motorAzimuth.getBusVoltage() * motorAzimuth.getAppliedOutput();
        inputs.azimuthCurrentAmps = motorAzimuth.getOutputCurrent();
        inputs.isAzimuthLimitSwitchPressed = azimuthLimitSwitch.get(); 
        
        inputs.pivotAngleDegrees = pivotServo.getAngle(); // Para el log
        
        inputs.flywheelVelocityRPMLider = motorLider.getEncoder().getVelocity();
        inputs.flywheelVelocityRPMFollower = motorSeguidor.getEncoder().getVelocity();
        inputs.flywheelAppliedVolts = motorLider.getBusVoltage() * motorLider.getAppliedOutput();
        inputs.flywheelCurrentAmps = motorLider.getOutputCurrent();
        inputs.flywheelTemplider = motorLider.getMotorTemperature();
        inputs.flywheeltempFollower = motorSeguidor.getMotorTemperature();
    }

    @Override
    public void setPID(double p, double i, double d) {
        // Actualizamos Slot 0 (Subir)
        configFlywheel.closedLoop.p(p);
        configFlywheel.closedLoop.i(i);
        configFlywheel.closedLoop.d(d);


        motorLider.configure(configFlywheel, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setAzimuthZero() {
        double currentRawDegrees = ( (azimuthTotalRotations + lastRawPosition) / kEncoderGearRatio ) * 360.0;
        this.angleOffset = currentRawDegrees;
    }

    @Override
    public void setAzimuthVoltage(double volts) {
        if (azimuthLimitSwitch.get() && volts > 0) { 
            motorAzimuth.setVoltage(0);
        } else {
            motorAzimuth.setVoltage(volts);
        }
    }
    
    @Override
    public void setPivotAngle(double degrees) {
        // Los servos estándar de FRC aceptan de 0 a 180 grados
        pivotServo.setAngle(degrees);
    }

    @Override
    public void setFlywheelVelocity(double rpm, double ffVolts) {
        // Le decimos: "Ve a estas RPM, y aquí tienes ffVolts de ayuda"
        controladorFlywheel.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
    }
    
    @Override
    public void setFlywheelVoltage(double volts) {
        motorLider.setVoltage(volts);
    }

    @Override
    public void stopAzimuth() {
        motorAzimuth.stopMotor();
    }
    
    @Override
    public void stopFlywheel() {
        motorLider.stopMotor();
    }
}