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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {
    
    // --- Motores Flywheel (Igual que siempre) ---
    private final SparkMax motorLider;
    private final SparkMax motorSeguidor;
    private final SparkClosedLoopController controladorFlywheel;

    // --- Componentes del Pivote ---
    private final SparkMax motorPivot;
    
    // Sensores conectados al ROBORIO
    private final DigitalInput limitSwitch; // DIO 1
    private final DutyCycleEncoder pivotEncoder; // DIO 2

    public ShooterIOReal() {
        // ---------------- CONFIGURACIÓN FLYWHEEL ----------------
        motorLider = new SparkMax(Constants.shooter.flywheels.kLiderID, MotorType.kBrushless);
        motorSeguidor = new SparkMax(Constants.shooter.flywheels.kSeguidorID, MotorType.kBrushless);
        controladorFlywheel = motorLider.getClosedLoopController();

        SparkMaxConfig configFlywheel = new SparkMaxConfig();
        configFlywheel.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        configFlywheel.closedLoop.p(Constants.shooter.flywheels.kP);
        configFlywheel.closedLoop.feedForward.kV(Constants.shooter.flywheels.kV);
        
        SparkMaxConfig configSeguidor = new SparkMaxConfig();
        configSeguidor.follow(motorLider, true); // Invertido

        motorLider.configure(configFlywheel, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorSeguidor.configure(configSeguidor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // ---------------- CONFIGURACIÓN PIVOTE ----------------
        motorPivot = new SparkMax(Constants.shooter.pivot.kID, MotorType.kBrushless);
        
        // Configuración básica del motor pivote
        SparkMaxConfig configPivot = new SparkMaxConfig();
        configPivot.idleMode(IdleMode.kBrake); // BRAKE ES OBLIGATORIO EN BRAZOS
        configPivot.smartCurrentLimit(40);     // Proteger el motor
        configPivot.inverted(false);           // Verifica esto físicamente
        
        motorPivot.configure(configPivot, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // --- Sensores en el RIO ---
        limitSwitch = new DigitalInput(1); // Conectado a DIO 1
        pivotEncoder = new DutyCycleEncoder(2); // Conectado a DIO 2 (Cable Absoluto/PWM del Through Bore)
        
        // CONFIGURACIÓN DE LA REDUCCIÓN 1:6
        // setDistancePerRotation define cuánto vale 1 vuelta del ENCODER en tus unidades (Grados del shooter).
        // 1 vuelta de encoder = (1 / 6) vueltas de shooter
        // (1.0 / 6.0) * 360.0 grados
        pivotEncoder.setDistancePerRotation((1.0 / 6.0) * 360.0);
        
        // Ajustamos la posición inicial a 0 (Asumiendo que inicias el robot con el shooter abajo)
        // Si inicias en otra posición, cambia el 0 por los grados iniciales.
        pivotEncoder.setPositionOffset(0); 
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Flywheel
        inputs.flywheelVelocityRPM = motorLider.getEncoder().getVelocity();
        inputs.flywheelAppliedVolts = motorLider.getBusVoltage() * motorLider.getAppliedOutput();
        inputs.flywheelCurrentAmps = motorLider.getOutputCurrent();

        // Pivote (Leído desde el RIO)
        // getDistance() ya nos dará los GRADOS gracias al setDistancePerRotation
        inputs.pivotPositionDegrees = pivotEncoder.getDistance(); 
        inputs.pivotAppliedVolts = motorPivot.getBusVoltage() * motorPivot.getAppliedOutput();
        inputs.pivotCurrentAmps = motorPivot.getOutputCurrent();
        
        // Seguridad: Leemos el Limit Switch
        // Normalmente, un switch conectado devuelve true si está abierto (no presionado) 
        // y false si se cierra a tierra (presionado), o viceversa dependiendo del cableado.
        // Asumiremos que devuelve TRUE cuando se presiona (ajusta el '!' si es al revés).
        inputs.isLimitSwitchPressed = limitSwitch.get(); 
    }

    @Override
    public void setFlywheelVelocity(double rpm) {
        controladorFlywheel.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    
    @Override
    public void setFlywheelVoltage(double volts) {
        motorLider.setVoltage(volts);
    }

    @Override
    public void setPivotVoltage(double volts) {
        // AQUÍ VA LA SEGURIDAD POR SOFTWARE
        // Si el limit switch está presionado Y intentamos movernos hacia el límite...
        // Asumimos que volts > 0 mueve hacia el limite. Si es al revés, cambia la lógica.
        if (limitSwitch.get() && volts > 0) { 
            motorPivot.setVoltage(0);
        } else {
            motorPivot.setVoltage(volts);
        }
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