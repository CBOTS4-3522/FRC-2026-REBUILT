package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {
    
    private final SparkMax motorLider;
    private final SparkMax motorSeguidor;
    private final SparkClosedLoopController controlador;

    public ShooterIOSparkMax() {
        // Inicializar motores con IDs de Constants
        motorLider = new SparkMax(Constants.shooter.kLiderID, MotorType.kBrushless);
        motorSeguidor = new SparkMax(Constants.shooter.kSeguidorID, MotorType.kBrushless);
        
        controlador = motorLider.getClosedLoopController();

        // --- Configuración LIDER ---
        SparkMaxConfig configLider = new SparkMaxConfig();
        configLider.idleMode(IdleMode.kCoast);
        configLider.inverted(false); 
        
        // PID y FeedForward (cargados desde Constants)
        configLider.closedLoop.p(Constants.shooter.kP);
        configLider.closedLoop.i(Constants.shooter.kI);
        configLider.closedLoop.d(Constants.shooter.kD);
        configLider.closedLoop.feedForward.kV(Constants.shooter.kV);
        configLider.closedLoop.feedForward.kA(Constants.shooter.kA);
        configLider.closedLoop.feedForward.kS(Constants.shooter.kS);

        // --- Configuración SEGUIDOR ---
        SparkMaxConfig configSeguidor = new SparkMaxConfig();
        configSeguidor.idleMode(IdleMode.kCoast);
        // Configurar el follower dentro del config object (Standard REV 2025/26)
        configSeguidor.follow(motorLider, true); // true = invertido respecto al líder

        // Aplicar configuraciones
        motorLider.configure(configLider, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorSeguidor.configure(configSeguidor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Leemos los datos del motor y los guardamos en el objeto inputs
        inputs.velocityRPM = motorLider.getEncoder().getVelocity();
        inputs.appliedVolts = motorLider.getBusVoltage() * motorLider.getAppliedOutput();
        inputs.currentAmps = motorLider.getOutputCurrent();
        inputs.tempCelcius = motorLider.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        motorLider.setVoltage(volts);
    }

    @Override
    public void setVelocity(double rpm) {
        controlador.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void stop() {
        motorLider.stopMotor();
    }
}