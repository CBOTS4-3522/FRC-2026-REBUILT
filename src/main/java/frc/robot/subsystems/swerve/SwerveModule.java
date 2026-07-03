/*
 * SwerveModule.java
 * 
 * Interfaz que define el contrato estándar para un módulo de tracción holonómica (Swerve).
 * Abstrae la implementación específica del hardware (REV, CTRE, etc.) para que el 
 * subsistema principal (SwerveBase) interactúe con los 4 módulos de manera uniforme.
 */
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    
    /**
     * Aplica el estado cinemático deseado al módulo.
     * @param desiredState Estado objetivo (Velocidad en m/s y Ángulo).
     * @param isOpenLoop Si es true, usa porcentaje de voltaje directo para tracción. Si es false, usa control PID de velocidad.
     * @param forceAngle Si es true, obliga al módulo a girar incluso si la velocidad de tracción es cercana a cero.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle);
    
    default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        setDesiredState(desiredState, isOpenLoop, false);
    }

    /** Obtiene la lectura absoluta del CANCoder (sin importar las vueltas del motor). */
    public Rotation2d getCanCoder();
    
    /** Obtiene el estado cinemático actual (Velocidad real y Ángulo real). */
    public SwerveModuleState getState();
    
    /** Obtiene la posición acumulada para la odometría (Distancia recorrida y Ángulo actual). */
    public SwerveModulePosition getPosition();
    
    /** Retorna el último estado objetivo que se le comandó al módulo. */
    public SwerveModuleState getDesiredState();
    
    public int getModuleNumber();
    public void setModuleNumber(int moduleNumber);
    
    /** 
     * Sincroniza el encoder relativo del motor de dirección (NEO) con la 
     * posición absoluta del CANCoder para evitar arrancar chuecos. 
     */
    public void synchronizeEncoders();
         
    /** Retorna la velocidad angular del módulo. */
    public double getOmega();
    
    /** Retorna el ángulo actual medido por el encoder relativo. */
    public Rotation2d getAngle();
     
    /** Inyecta voltaje directo al motor de tracción (Utilizado para SysId). */
    public void setDriveVoltage(double volts);
         
    /** Retorna el voltaje aplicado actualmente al motor de tracción. */
    public double getDriveVoltage();
         
    /** Bloquea la dirección del módulo a 0 grados (Usado en rutinas de caracterización). */
    public void lockAngle();
}