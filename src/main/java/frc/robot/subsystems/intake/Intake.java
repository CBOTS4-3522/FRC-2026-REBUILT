package frc.robot.subsystems.intake;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Intake extends SubsystemBase {
    
    private final SparkMax motor;

    public Intake() {
        motor = new SparkMax(Constants.Intake.kMotorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        
        // Usamos la constante de IdleMode que ya tenías en SwerveConstants o definimos una aquí
        // Usualmente Brake es mejor para pruebas, Coast para operación normal si no quieres que se trabe
        config.idleMode(IdleMode.kBrake); 
        
        config.inverted(false); // Cambiar a true si gira al revés
        
        // Limitamos la corriente para proteger el mecanismo (ej. 40 Amps)
        config.smartCurrentLimit(40);

        // Aplicamos la configuración (API 2026)
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setPorcentaje(double porcentaje) {
        motor.set(porcentaje);
    }
    
    public void stop() {
        motor.stopMotor();
    }
    
    @Override
    public void periodic() {
        // Opcional: Publicar temperatura o corriente a Elastic si gustas
    }
}