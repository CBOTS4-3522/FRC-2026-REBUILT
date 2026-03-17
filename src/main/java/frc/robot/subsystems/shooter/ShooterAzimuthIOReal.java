package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Servo;

public class ShooterAzimuthIOReal implements ShooterAzimuthIO {

    private final Servo pivotServo; // Mantenemos vivo el chamfle

    public ShooterAzimuthIOReal() {
        // --- SERVO PIVOT ---
        pivotServo = new Servo(0);
    }

    @Override
    public void updateInputs(ShooterAzimuthIOInputs inputs) {
        inputs.pivotAngleDegrees = pivotServo.getAngle();
    }

    @Override
    public void setPivotAngle(double degrees) {
        pivotServo.setAngle(degrees);
    }

    // ==========================================================
    // MÉTODOS MUERTOS (Para no romper la interfaz)
    // ==========================================================
    @Override public void setAzimuthVoltage(double volts) {}
    @Override public void stopAzimuth() {}
    @Override public void setAzimuthPosition(double degrees) {}
    @Override public void setAzimuthZero() {}
}