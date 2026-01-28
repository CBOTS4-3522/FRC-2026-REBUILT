package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Shooter extends SubsystemBase {
    
    private final SparkMax motorLider;
    private final SparkMax motorSeguidor;
    
    // Aquí pondremos los IDs correctos después
    private final int liderID = Constants.shooter.kLiderID; 
    private final int seguidorID = Constants.shooter.kSeguidorID; 

    public Shooter() {
        motorLider = new SparkMax(liderID, MotorType.kBrushless);
        motorSeguidor = new SparkMax(seguidorID, MotorType.kBrushless);

        // Configuración del LIDER
        SparkMaxConfig configLider = new SparkMaxConfig();
        configLider.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast); // Coast es mejor para shooters
        configLider.inverted(false); // Cambiar si gira al revés
        

        // Valores iniciales (luego los ajustaremos en Elastic)
        configLider.closedLoop.p(0.0001); 
        configLider.closedLoop.i(0);
        configLider.closedLoop.d(0);
        configLider.closedLoop.feedForward.kV(0.00018); // Ayuda a mantener velocidad base

        // Configuración del SEGUIDOR
        SparkMaxConfig configSeguidor = new SparkMaxConfig();
        configSeguidor.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        
        // Esta línea le dice al seguidor que copie al líder, pero INVERTIDO (true)
        // Nota: En la API 2026 a veces se configura directo en el follow, 
        // pero vamos a usar la configuración estándar de REV para esto.
        configSeguidor.follow(motorLider, true); 

        // Aplicamos las configuraciones
        motorLider.configure(configLider, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motorSeguidor.configure(configSeguidor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Método simple para probar voltaje (0 a 1)
    public void setPorcentaje(double porcentaje) {
        motorLider.set(porcentaje);
    }
    
    // Método para detener
    public void stop() {
        motorLider.stopMotor();
    }
    
    // Método para obtener la velocidad actual (para verla en Elastic)
    public double getVelocidad() {
        return motorLider.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        // Aquí publicaremos datos a Elastic más adelante
    }
}