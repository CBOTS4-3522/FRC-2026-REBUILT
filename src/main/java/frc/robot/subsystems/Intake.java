package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.lang.module.ModuleDescriptor.Requires;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Intake extends SubsystemBase {

    private final SparkMax motor;
    private final VictorSPX motorUD;
    private static final String KEY_VELOCIDADI = "Intake/VelocidadI";
    private static final String KEY_VELOCIDADU = "Intake/UP";
    private static final String KEY_VELOCIDADD = "Intake/DOWN";

    public Intake() {

        
        motor = new SparkMax(Constants.Intake.kMotorID, MotorType.kBrushless);
        motorUD = new VictorSPX(Constants.Intake.rMotorID);

        SparkMaxConfig config = new SparkMaxConfig();

        // CONFIGURACIÓN
        // Usamos la constante de IdleMode que ya tenías en SwerveConstants o definimos
        // una aquí
        // Usualmente Brake es mejor para pruebas, Coast para operación normal si no
        // quieres que se trabe
        config.idleMode(IdleMode.kCoast);

        config.inverted(false); // Cambiar a true si gira al revés

        // Limitamos la corriente para proteger el mecanismo (ej. 40 Amps)
        config.smartCurrentLimit(50);

        // Aplicamos la configuración (API 2026)
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // JALAR
    public void setPorcentaje(double porcentaje) {
        motor.set(porcentaje);
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command runIntake() {
        return run(
                () -> {
                    // Lee el valor de Elastic (Default 0.0)
                    double velocidad = SmartDashboard.getNumber(KEY_VELOCIDADI, 0);

                    velocidad = MathUtil.clamp(velocidad, -1.0, 1.0);

                    setPorcentaje(velocidad);
                }).finallyDo(
                        interrupted -> {
                            stop();
                        });
    }

    

    // SUBIR
    public Command up() {
        return run(
                () -> {
                    // Lee el valor de Elastic (Default 0.0)
                    double velocidad2 = SmartDashboard.getNumber(KEY_VELOCIDADU, 0);

                    velocidad2 = MathUtil.clamp(velocidad2, 0, 1.0);

                    motorUD.set(ControlMode.PercentOutput, velocidad2);
                }).finallyDo(
                        interrupted -> {
                            motorUD.set(ControlMode.PercentOutput, 0);
                        });
    }

    // BAJAR
    public Command down() {
        return run(
                () -> {
                    // Lee el valor de Elastic (Default 0.0)
                    double velocidad3 = SmartDashboard.getNumber(KEY_VELOCIDADD, 0);

                    velocidad3 = MathUtil.clamp(velocidad3, -1, 0);
                    motorUD.set(ControlMode.PercentOutput, velocidad3);
                }).finallyDo(
                        interrupted -> {
                            motorUD.set(ControlMode.PercentOutput, 0);
                        });
    }

    // autocommands
    // subir
    public Command upAuto(){
        return this.runEnd(
            () -> motorUD.set(ControlMode.PercentOutput, 1),
            () -> motorUD.set(ControlMode.PercentOutput, 0)
         ).withTimeout(0.5);
    }

    public Command downAuto(){
        return this.runEnd(
            () -> motorUD.set(ControlMode.PercentOutput, -1),
            () -> motorUD.set(ControlMode.PercentOutput, 0)
         ).withTimeout(0.5);
    }

    @Override
    public void periodic() {
        // Opcional: Publicar temperatura o corriente a Elastic si gustas
        SmartDashboard.putNumber("Intake/Corriente", motor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Temperatura", motor.getMotorTemperature());

    }
}