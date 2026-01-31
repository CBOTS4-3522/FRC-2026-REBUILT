package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;



public class Shooter extends SubsystemBase {
    
    private final SparkMax motorLider;
    private final SparkMax motorSeguidor;
    
    // Aquí pondremos los IDs correctos después
    private final int liderID = Constants.shooter.kLiderID; 
    private final int seguidorID = Constants.shooter.kSeguidorID; 
    private static final String KEY_RPM = "Shooter/ObjetivoRPM";

    public Shooter() {
        motorLider = new SparkMax(liderID, MotorType.kBrushless);
        motorSeguidor = new SparkMax(seguidorID, MotorType.kBrushless);
        SmartDashboard.putNumber(KEY_RPM, 1500);

        // Configuración del LIDER
        SparkMaxConfig configLider = new SparkMaxConfig();
        configLider.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast); // Coast es mejor para shooters
        configLider.inverted(false); // Cambiar si gira al revés
        

        // Valores iniciales (luego los ajustaremos en Elastic)
        configLider.closedLoop.p(Constants.shooter.kP);
        configLider.closedLoop.i(Constants.shooter.kI);
        configLider.closedLoop.d(Constants.shooter.kD);
        configLider.closedLoop.feedForward.kV(Constants.shooter.kV); 
        configLider.closedLoop.feedForward.kA(Constants.shooter.kA);
        configLider.closedLoop.feedForward.kS(Constants.shooter.kS);


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

    
    // Método para controlar por VELOCIDAD (RPM)
public void setVelocidad(double rpm) {
    // Obtenemos el controlador interno del motor
    SparkClosedLoopController controlador = motorLider.getClosedLoopController();

    // Le ordenamos ir a las RPM deseadas usando el Slot 0 (donde guardamos el PID)
    controlador.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
}

    @Override
    public void periodic() {
        // Aquí publicaremos datos a Elastic más adelante
    }
}