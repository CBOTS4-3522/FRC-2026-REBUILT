/*
 * ShooterFlywheelsIOReal.java
 *
 * Implementación de hardware para los Flywheels utilizando un esquema Líder-Seguidor.
 * Esto reduce drásticamente la latencia y el tráfico en el bus CAN, ya que 
 * el controlador esclavo imita las salidas de voltaje del maestro internamente.
 */
package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ShooterFlywheelsIOReal implements ShooterFlywheelsIO {
    private final SparkMax motorLider;
    private final SparkMax motorSeguidor;
    
    // Controlador de lazo cerrado interno procesado nativamente por el SparkMax (1 kHz)
    private final SparkClosedLoopController controladorFlywheel;
    SparkMaxConfig configFlywheel = new SparkMaxConfig();

    public ShooterFlywheelsIOReal() {
        motorLider = new SparkMax(Constants.shooter.flywheels.kLiderID, MotorType.kBrushless);
        motorSeguidor = new SparkMax(Constants.shooter.flywheels.kSeguidorID, MotorType.kBrushless);
        controladorFlywheel = motorLider.getClosedLoopController();
        
        // --- CONFIGURACIÓN MOTOR LÍDER ---
        configFlywheel.idleMode(IdleMode.kBrake);
        configFlywheel.inverted(false);
        configFlywheel.closedLoopRampRate(0.5); // Rampa suave para evitar tirones de corriente altos
        configFlywheel.closedLoop.p(Constants.shooter.flywheels.kP);
        configFlywheel.closedLoop.i(Constants.shooter.flywheels.kI);
        configFlywheel.closedLoop.d(Constants.shooter.flywheels.kD);
        
        // --- CONFIGURACIÓN MOTOR SEGUIDOR ---
        SparkMaxConfig configSeguidor = new SparkMaxConfig();
        configSeguidor.idleMode(IdleMode.kBrake);
        configSeguidor.follow(motorLider, true); // True indica que girará físicamente al lado contrario
        
        motorLider.configure(configFlywheel, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorSeguidor.configure(configSeguidor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(ShooterFlywheelsIOInputs inputs) {
        inputs.flywheelVelocityRPMLider = motorLider.getEncoder().getVelocity();
        inputs.flywheelVelocityRPMFollower = motorSeguidor.getEncoder().getVelocity();
        inputs.flywheelAppliedVolts = motorLider.getBusVoltage() * motorLider.getAppliedOutput();
        inputs.flywheelCurrentAmps = motorLider.getOutputCurrent();
        inputs.flywheelTemplider = motorLider.getMotorTemperature();
        inputs.flywheeltempFollower = motorSeguidor.getMotorTemperature();
        inputs.errorEncoderLider = motorLider.getFaults().sensor;
        inputs.errorEncoderFollower = motorSeguidor.getFaults().sensor;
    }

    @Override
    public void setPID(double p, double i, double d) {
        configFlywheel.closedLoop.p(p);
        configFlywheel.closedLoop.i(i);
        configFlywheel.closedLoop.d(d);
        motorLider.configure(configFlywheel, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setFlywheelVelocity(double rpm, double ffVolts) {
        // Enlaza la meta de RPM con el FeedForward de soporte
        controladorFlywheel.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        motorLider.setVoltage(volts);
    }

    @Override
    public void stopFlywheel() {
        motorLider.stopMotor();
    }
}