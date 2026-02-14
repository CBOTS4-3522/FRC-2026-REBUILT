package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {
    
    // --- Motores ---
    private final SparkMax motorLider;
    private final SparkMax motorSeguidor;
    private final SparkClosedLoopController controladorFlywheel;
    private final SparkMax motorPivot;
    
    // --- Sensores RIO ---
    private final DigitalInput limitSwitch; // DIO 1
    private final DutyCycleEncoder pivotEncoder; // DIO 2

    // --- VARIABLES DE CÁLCULO (Software Encoder) ---
    private double lastRawPosition = 0.0; 
    private double pivotTotalRotations = 0.0; 
    
    // Reducción: 1 vuelta de brazo = 6 vueltas de encoder
    private final double kEncoderGearRatio = 6.0; 
    
    // Offset: Ajuste para definir el cero
    private double angleOffset = 0.0;

    public ShooterIOSparkMax() {
        // --- CONFIGURACIÓN FLYWHEELS (Igual) ---
        motorLider = new SparkMax(Constants.shooter.flywheels.kLiderID, MotorType.kBrushless);
        motorSeguidor = new SparkMax(Constants.shooter.flywheels.kSeguidorID, MotorType.kBrushless);
        controladorFlywheel = motorLider.getClosedLoopController();

        SparkMaxConfig configFlywheel = new SparkMaxConfig();
        configFlywheel.idleMode(IdleMode.kBrake);
        configFlywheel.closedLoop.p(Constants.shooter.flywheels.kP);
        configFlywheel.closedLoop.feedForward.kV(Constants.shooter.flywheels.kV);
 
        
        SparkMaxConfig configSeguidor = new SparkMaxConfig();
        configSeguidor.follow(motorLider, true);

        motorLider.configure(configFlywheel, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorSeguidor.configure(configSeguidor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // --- CONFIGURACIÓN PIVOTE ---
        motorPivot = new SparkMax(Constants.shooter.pivot.kID, MotorType.kBrushless);
        
        SparkMaxConfig configPivot = new SparkMaxConfig();
        configPivot.idleMode(IdleMode.kBrake); 
        configPivot.smartCurrentLimit(40);     
        configPivot.inverted(false);           
        
        motorPivot.configure(configPivot, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // --- SENSORES ---
        limitSwitch = new DigitalInput(1);
        pivotEncoder = new DutyCycleEncoder(2); 
        
        // INICIALIZACIÓN
        // Al arrancar, asumimos que "lo que lee el encoder AHORA" es nuestro punto de partida.
        // Pero OJO: Si el robot arranca con el shooter levantado, esto marcará 0 ahí.
        // Lo ideal es arrancar siempre abajo
        lastRawPosition = pivotEncoder.get();
        
        // Si tuviéramos un valor conocido de "Home" en Constants, lo aplicaríamos aquí.
        // Por ahora, asumimos que 0 = Posición de encendido.
        angleOffset = 0.0; 
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // 1. Leer valor crudo (0.0 a 1.0 siempre en 2025)
        double currentRaw = pivotEncoder.get();
        
        // 2. Detectar vueltas completas (Rollover)
        double diff = currentRaw - lastRawPosition;
        
        if (diff < -0.5) {
            pivotTotalRotations += 1.0; // Pasó de 0.9 a 0.1 -> Sumar vuelta
        } else if (diff > 0.5) {
            pivotTotalRotations -= 1.0; // Pasó de 0.1 a 0.9 -> Restar vuelta
        }
        
        lastRawPosition = currentRaw;
        
        // 3. Calcular vueltas totales ABSOLUTAS del encoder
        double totalEncoderTurns = pivotTotalRotations + currentRaw;
        
        // 4. Convertir a GRADOS del BRAZO
        // Grados = (VueltasEncoder / Ratio) * 360
        double rawDegrees = (totalEncoderTurns / kEncoderGearRatio) * 360.0;
        
        // 5. Aplicar Offset y guardar en inputs
        inputs.pivotPositionDegrees = rawDegrees - angleOffset;

        // --- Resto de inputs ---
        inputs.pivotAppliedVolts = motorPivot.getBusVoltage() * motorPivot.getAppliedOutput();
        inputs.pivotCurrentAmps = motorPivot.getOutputCurrent();
        inputs.isLimitSwitchPressed = limitSwitch.get(); 
        
        inputs.flywheelVelocityRPM = motorLider.getEncoder().getVelocity();
        inputs.flywheelAppliedVolts = motorLider.getBusVoltage() * motorLider.getAppliedOutput();
        inputs.flywheelCurrentAmps = motorLider.getOutputCurrent();
    }

    // Método extra para definir el Cero manualmente (puedes llamarlo desde un comando)
    public void setPivotZero() {
        // Calculamos cuánto vale el ángulo "bruto" ahora mismo
        double currentRawDegrees = ( (pivotTotalRotations + lastRawPosition) / kEncoderGearRatio ) * 360.0;
        
        // Decimos que el offset es igual a ese valor
        // Entonces: Posición = ValorActual - Offset = 0
        this.angleOffset = currentRawDegrees;
    }

    @Override
    public void setPivotVoltage(double volts) {
        // Seguridad con Limit Switch (asumiendo switch cierra a true)
        // Y asumiendo que voltaje positivo (+) mueve hacia el switch
        if (limitSwitch.get() && volts > 0) { 
            motorPivot.setVoltage(0);
        } else {
            motorPivot.setVoltage(volts);
        }
    }
    
    // ... Resto de métodos (flywheel, stop, etc) ...
    @Override
    public void setFlywheelVelocity(double rpm) {
        controladorFlywheel.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    
    @Override
    public void setFlywheelVoltage(double volts) {
        motorLider.setVoltage(volts);
    }

    @Override
    public void stopPivot() {
        motorPivot.stopMotor();
    }
    
    @Override
    public void stopFlywheel() {
        motorLider.stopMotor();
    }
}