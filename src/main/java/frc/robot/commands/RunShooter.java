package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
    private final Shooter s_Shooter;
    
    // Cambiamos el nombre de la variable en Elastic para no confundirnos
    private static final String KEY_RPM = "Shooter/ObjetivoRPM";

    public RunShooter(Shooter shooter) {
        this.s_Shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("Shooter PID Iniciado");
        // Publicamos un valor por defecto si no existe (ej. 1500 RPM para probar)
        
    }

    @Override
    public void execute() {
        // 1. Leemos el valor de RPM desde Elastic
        double rpmObjetivo = SmartDashboard.getNumber(KEY_RPM, 0.0);

        // 2. Protección: Los NEOs giran máx a ~5676 RPM.
        // Limitamos para no pedir imposibles.
        if (rpmObjetivo > 5500) rpmObjetivo = 5500;
        if (rpmObjetivo < 0) rpmObjetivo = 0; // Asumimos que no queremos reversa en PID

        // 3. Mandamos la señal de VELOCIDAD
        s_Shooter.setVelocidad(rpmObjetivo);
        
        // 4. (Opcional) Publicar la velocidad real para comparar en Elastic
        SmartDashboard.putNumber("Shooter/VelocidadReal", s_Shooter.getVelocidad());
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.stop();
        System.out.println("Shooter Detenido");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}