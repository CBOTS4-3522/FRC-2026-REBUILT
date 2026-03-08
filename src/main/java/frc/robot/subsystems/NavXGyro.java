package frc.robot.subsystems; 

// Importaciones específicas de STUDICA
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public class NavXGyro extends AHRS {

    private static NavXGyro instance;
    public static double zeroHeading;
    public static double zeroAngle;
    private double simAngle = 0.0;

    private NavXGyro() {
        // En la librería de Studica, se usa NavXComType, no SPI.Port
        super(NavXComType.kMXP_SPI);

        // Espera de seguridad para inicialización
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        zeroHeading = getNavHeading(); 
        zeroAngle = getNavAngle();
        System.out.println("Setup ZeroAngle " + zeroAngle);
    }

    public static NavXGyro getInstance() {
        if (instance == null) {
            instance = new NavXGyro();
        }
        return instance;
    }

    public double getNavHeading() {
        // En Studica a veces devuelve float, aseguramos el cast a double
        return (double) getFusedHeading();
    }

    public double getNavAngle() {
        return (double) getAngle();
    }

    public void zeroNavHeading() {
        reset();
        zeroHeading = getNavHeading();
        zeroAngle = getNavAngle();
    }

    public double getZeroHeading(){
        return zeroHeading;
    }

    public double getZeroAngle(){
        return zeroAngle;
    }

    public void updateSimAngle(double dtSeconds, double radiansPerSecond){
        // Convertimos de radianes a grados y sumamos
        double degreesPerSecond = Math.toDegrees(radiansPerSecond);
        simAngle += degreesPerSecond * dtSeconds;
        
        // Normalizamos entre 0 y 360 si quieres, aunque Rotation2d lo maneja solo
    }
    

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    @Override
    public Rotation2d getRotation2d() {
        // 3. Si es simulación, regresamos la variable fantasma
        if (RobotBase.isSimulation()) {
            // En simulación usualmente CCW es positivo directo
            return Rotation2d.fromDegrees(simAngle);
        }
        // Si es real, usamos la NavX
        return Rotation2d.fromDegrees(-getNavAngle());
    }
}