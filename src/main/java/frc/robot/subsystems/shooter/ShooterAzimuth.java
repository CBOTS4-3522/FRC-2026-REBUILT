package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAzimuth extends SubsystemBase {

    private final ShooterAzimuthIO io;
    private final ShooterAzimuthIOInputsAutoLogged inputs = new ShooterAzimuthIOInputsAutoLogged();
    private double anguloChamfleManual = 0.0;

    public ShooterAzimuth(ShooterAzimuthIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Azimuth", inputs);
    }

    public void setObjetivo(double targetChamfle) {
        io.setPivotAngle(targetChamfle);
    }

    public Command controlManualAzimuth(DoubleSupplier ejeAzimuth, DoubleSupplier ejeChamfle) {
        return this.run(() -> {
            // El ejeAzimuth (torreta) ya no hace nada. Solo leemos el chamfle.
            double valChamfle = MathUtil.applyDeadband(ejeChamfle.getAsDouble(), 0.1);
            anguloChamfleManual += valChamfle * 1.5;
            anguloChamfleManual = MathUtil.clamp(anguloChamfleManual, 0.0, 180.0);
            io.setPivotAngle(anguloChamfleManual);
        });
    }

    // ==========================================================
    // FUNCIONES FANTASMA (Para que RobotContainer no marque error)
    // ==========================================================
    public boolean isTorretaEnTope() { return false; }
    public Command resetAzimuthEncoder() { return runOnce(() -> {}); }
    public Command homingCero() { return runOnce(() -> {}); }
    public Command probarFriccionEstatica() { return runOnce(() -> {}); }
    public Command autoApuntar(DoubleSupplier angulo) { return runOnce(() -> {}); }
    public void detener() {}
}