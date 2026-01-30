package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {
    private final Intake s_Intake;
    
    // Llave para Elastic
    private static final String KEY_VELOCIDAD = "Intake/VelocidadTest";

    public RunIntake(Intake intake) {
        this.s_Intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("Intake Iniciado");
    }

    @Override
    public void execute() {
        // Lee el valor de Elastic (Default 0.0)
        double velocidad = SmartDashboard.getNumber(KEY_VELOCIDAD, 1.0);

        // Protecciones básicas
        if (velocidad > 1.0) velocidad = 1.0;
        if (velocidad < -1.0) velocidad = -1.0;

        s_Intake.setPorcentaje(velocidad);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stop();
        System.out.println("Intake Detenido");
    }

    @Override
    public boolean isFinished() {
        return false; // Funciona como Toggle (hasta que se apague el botón)
    }
}