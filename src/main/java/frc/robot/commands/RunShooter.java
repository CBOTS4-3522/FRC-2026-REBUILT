package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
    private final Shooter s_Shooter;
    
    // Usaremos esta clave para encontrar el valor en Elastic
    private static final String KEY_VELOCIDAD = "Shooter/VelocidadTest";

    public RunShooter(Shooter shooter) {
        this.s_Shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Opcional: Podrías imprimir algo aquí para saber que arrancó
        System.out.println("Shooter Iniciado");
    }

    @Override
    public void execute() {
        // 1. Leemos el valor desde Elastic (SmartDashboard)
        // El 0.0 es el valor por defecto si no encuentra nada
        double velocidad = SmartDashboard.getNumber(KEY_VELOCIDAD, 0.0);

        // 2. Proteccion: Aseguramos que no pase de 1.0 (100%)
        if (velocidad > 1.0) velocidad = 1.0;
        if (velocidad < -1.0) velocidad = -1.0;

        // 3. Mandamos el voltaje
        s_Shooter.setPorcentaje(velocidad);
    }

    @Override
    public void end(boolean interrupted) {
        // Cuando apagamos el toggle, detenemos los motores
        s_Shooter.stop();
        System.out.println("Shooter Detenido");
    }

    @Override
    public boolean isFinished() {
        // Queremos que siga corriendo hasta que apaguemos el botón (Toggle)
        return false;
    }
}