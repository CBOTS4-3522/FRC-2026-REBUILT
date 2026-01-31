package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Intake extends SubsystemBase {
    
    private final SparkMax motor;
    private static final String KEY_VELOCIDAD = "Intake/VelocidadTest";

    public Intake() {
        motor = new SparkMax(Constants.Intake.kMotorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        
        // Usamos la constante de IdleMode que ya tenías en SwerveConstants o definimos una aquí
        // Usualmente Brake es mejor para pruebas, Coast para operación normal si no quieres que se trabe
        config.idleMode(IdleMode.kCoast); 
        
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

    public Command runIntake(){
        return run(
            ()-> {
                // Lee el valor de Elastic (Default 0.0)
        double velocidad = SmartDashboard.getNumber(KEY_VELOCIDAD, 1.0);

        // Protecciones básicas
        if (velocidad > 1.0) velocidad = 1.0;
        if (velocidad < -1.0) velocidad = -1.0;

        setPorcentaje(velocidad);
            }
        ).finallyDo(
            interrupted -> {
                stop();
            }
            )
        ;
    }
    
    @Override
    public void periodic() {
        // Opcional: Publicar temperatura o corriente a Elastic si gustas
    }
}