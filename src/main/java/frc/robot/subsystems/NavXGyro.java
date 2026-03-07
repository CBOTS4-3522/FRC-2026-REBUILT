package frc.robot.subsystems; 

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType; // Asegúrate de que el enum kUSB1 esté importado correctamente
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public class NavXGyro {

    private static NavXGyro instance;
    
    // En lugar de heredar, creamos la instancia aquí adentro (Composición)
    private AHRS ahrs; 

    public static double zeroHeading;
    public static double zeroAngle;
    private double simAngle = 0.0;

    private NavXGyro() {
        System.out.println("Iniciando conexión con NavX...");
        
        // 1. Intentamos conectar por SPI primero
        ahrs = new AHRS(NavXComType.kMXP_SPI);
        
        // Espera de seguridad para que el sensor bootee
        esperar(100);

        // 2. Lógica de Fallback: Si no hay conexión, cambiamos a USB
        if (!ahrs.isConnected()) {
            System.out.println("ALERTA: Fallo en el puerto SPI. Intentando conexión por USB...");
            
            // Re-instanciamos usando el puerto USB 
            // (Verifica si en Studica es kUSB1 o kUSB)
            ahrs = new AHRS(NavXComType.kUSB1); 
            esperar(100);
        }

        // 3. Verificación final
        if (ahrs.isConnected()) {
            System.out.println("✅ NavX conectado exitosamente.");
        } else {
            System.err.println("❌ ERROR CRITICO: El NavX no responde ni por SPI ni por USB.");
        }

        zeroHeading = getNavHeading(); 
        zeroAngle = getNavAngle();
        System.out.println("Setup ZeroAngle: " + zeroAngle);
    }

    public static NavXGyro getInstance() {
        if (instance == null) {
            instance = new NavXGyro();
        }
        return instance;
    }

    // Método auxiliar para limpiar el Thread.sleep
    private void esperar(int milisegundos) {
        try {
            Thread.sleep(milisegundos);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public double getNavHeading() {
        // Ahora llamamos a los métodos a través de nuestro objeto 'ahrs'
        return (double) ahrs.getFusedHeading();
    }

    public double getNavAngle() {
        return (double) ahrs.getAngle();
    }

    public double getRoll() {
        return (double) ahrs.getRoll();
    }


    public void zeroNavHeading() {
        ahrs.reset();
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
        double degreesPerSecond = Math.toDegrees(radiansPerSecond);
        simAngle += degreesPerSecond * dtSeconds;
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public Rotation2d getRotation2d() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromDegrees(simAngle);
        }
        return Rotation2d.fromDegrees(-getNavAngle());
    }
    
    // Te agrego este método por si la odometría necesita saber si el giroscopio está vivo
    public boolean isConnected() {
        return ahrs.isConnected();
    }
}