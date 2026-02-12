package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    
    private final SysIdRoutine m_sysIdRoutine;

    // Constructor que recibe la implementación IO (Inyección de dependencias)
    public Shooter(ShooterIO io) {
        this.io = io;

        // Configuración de SysId
        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> io.setVoltage(volts.in(Volts)), // Manda voltaje a través de IO
                log -> {
                    // Loggea datos usando los inputs de AdvantageKit (ya leídos)
                    log.motor("shooter-flywheel")
                        .voltage(Volts.of(inputs.appliedVolts))
                        .angularVelocity(RPM.of(inputs.velocityRPM))
                        .angularPosition(Rotations.of(0)); // Posición no es crítica en shooter de velocidad, pero se puede agregar
                },
                this
            )
        );
    }

    @Override
    public void periodic() {
        // 1. Actualizar inputs desde el hardware (o simulación)
        io.updateInputs(inputs);
        
        // 2. Registrar todo en AdvantageScope
        Logger.processInputs("Shooter", inputs);
    }

    /** Comanda el shooter a una velocidad específica */
    public Command runShooterCommand(double rpm) {
        return this.run(() -> io.setVelocity(rpm));
    }
    
    /** Detiene el shooter */
    public Command stopCommand() {
        return this.runOnce(() -> io.stop());
    }

    // Comandos de SysId
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}