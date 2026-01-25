package frc.robot.subsystems; 

// Importaciones específicas de STUDICA
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class NavXGyro extends AHRS {

    private static NavXGyro instance;
    public static double zeroHeading;
    public static double zeroAngle;

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
    
    @Override
    public Rotation2d getRotation2d() {
        // Invertimos el signo para el sistema de coordenadas de WPILib (CCW+)
        return Rotation2d.fromDegrees(-getNavAngle());
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }
}