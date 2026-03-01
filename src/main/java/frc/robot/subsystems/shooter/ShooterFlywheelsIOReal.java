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

    // --- Motores y Sensores ---
    private final SparkMax motorLider;
    private final SparkMax motorSeguidor;
    private final SparkClosedLoopController controladorFlywheel;

    SparkMaxConfig configFlywheel = new SparkMaxConfig();

    public ShooterFlywheelsIOReal() {
        // --- CONFIGURACIÓN FLYWHEELS ---
        motorLider = new SparkMax(Constants.shooter.flywheels.kLiderID, MotorType.kBrushless);
        motorSeguidor = new SparkMax(Constants.shooter.flywheels.kSeguidorID, MotorType.kBrushless);
        controladorFlywheel = motorLider.getClosedLoopController();

        // Motor líder
        configFlywheel.idleMode(IdleMode.kBrake);
        configFlywheel.inverted(true);
        configFlywheel.closedLoopRampRate(0.5);
        configFlywheel.closedLoop.p(Constants.shooter.flywheels.kP);
        configFlywheel.closedLoop.i(Constants.shooter.flywheels.kI);
        configFlywheel.closedLoop.d(Constants.shooter.flywheels.kD);

        // Motor Esclavo
        SparkMaxConfig configSeguidor = new SparkMaxConfig();
        configSeguidor.idleMode(IdleMode.kBrake);
        configSeguidor.follow(motorLider, true);

        // Configuraciones
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
    public void setFlywheelVelocity(double rpm, double ffVolts) {
        // Le decimos: "Ve a estas RPM, y aquí tienes ffVolts de ayuda"
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