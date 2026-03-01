package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.units.measure.;

// Importaciones de PathPlanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;

import frc.robot.Constants;
import frc.robot.subsystems.NavXGyro;

public class SwerveBase extends SubsystemBase {

    private double lastDriveKP = Constants.Swerve.Drive.kP;
    private double lastDriveKD = Constants.Swerve.Drive.kD;
    
    public SwerveDrivePoseEstimator swerveOdometer;
    public SwerveModule[] swerveMods;
    public NavXGyro gyro = NavXGyro.getInstance();

    private final Field2d field = new Field2d();
    private RobotConfig config;

    public SwerveBase() {

        // cajas para calibrar pid drive
        SmartDashboard.putNumber("Turning/Drive kP", Constants.Swerve.Drive.kP);
        SmartDashboard.putNumber("Tuning/Drive kD", Constants.Swerve.Drive.kD);

        SmartDashboard.putNumber("SysId/Tiempo Quasistatic", 7.0); // Necesita más tiempo
        SmartDashboard.putNumber("SysId/Tiempo Dynamic", 1.5); // Necesita poco tiempo

        SmartDashboard.putNumber("Meneito/Amplitud", 1.0);
        SmartDashboard.putNumber("Meneito/Frecuencia", 10);

        swerveMods = new RevSwerveModule[] {
                new RevSwerveModule(0, Constants.Swerve.Mod0.kConstants),
                new RevSwerveModule(1, Constants.Swerve.Mod1.kConstants),
                new RevSwerveModule(2, Constants.Swerve.Mod2.kConstants),
                new RevSwerveModule(3, Constants.Swerve.Mod3.kConstants)
        };

        swerveOdometer = new SwerveDrivePoseEstimator(
                Constants.Swerve.kSwerveKinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d());

        zeroGyro();

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,

                (speeds, feedforwards) -> {
                    ChassisSpeeds fixedSpeeds = new ChassisSpeeds(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
                    driveRobotRelative(fixedSpeeds);
                },

                new PPHolonomicDriveController(
                        new PIDConstants(7.0, 0.0, 0.0), //traslacion
                        new PIDConstants(5.0, 0.0, 0.0) ), //rotacion
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

    }

    // Método principal para Teleoperado
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        Rotation2d currentYaw = getYaw();

        ChassisSpeeds desiredChassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                currentYaw)
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);

        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.kSwerveKinematics
                .toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeed);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Constants.Swerve.kSwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.kMaxSpeed);
        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeed);
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometer.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometer.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro() {
        gyro.zeroNavHeading();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.kInvertGyro) ? Rotation2d.fromDegrees(360).minus(gyro.getRotation2d())
                : gyro.getRotation2d();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    // --- UTILIDADES ---

    public void wheelsIn() {
        swerveMods[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false, true);
        swerveMods[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)), false,true);
        swerveMods[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false,true);
        swerveMods[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-135)), false,true);
    }

    public void stop() {
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(new SwerveModuleState(0, mod.getState().angle), false);
        }
    }

    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose = new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = new Pose2d().log(futureRobotPose);
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    // Sysld
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, Seconds.of(30.0), null),
            new SysIdRoutine.Mechanism(
                    (voltage) -> { // 'voltage' aquí es un objeto de tipo Voltage
                        for (SwerveModule mod : swerveMods) {
                            mod.setDriveVoltage(voltage.in(Volts)); // Convertimos el objeto a número (double)
                            mod.lockAngle();
                        }
                    },
                    log -> {
                        for (SwerveModule mod : swerveMods) {
                            log.motor("drive-" + mod.getModuleNumber())
                                    .voltage(Volts.of(mod.getDriveVoltage())) // Creamos la medida a partir del número
                                    .linearPosition(Meters.of(mod.getPosition().distanceMeters))
                                    .linearVelocity(MetersPerSecond.of(mod.getState().speedMetersPerSecond));
                        }
                    },
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction)
                .withTimeout(Seconds.of(SmartDashboard.getNumber("SysId/Tiempo Quasistatic", 7.0)));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction)
                .withTimeout(Seconds.of(SmartDashboard.getNumber("SysId/Tiempo Dynamic", 1.5)));
    }


//Vision Measurement
   public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        
        // Obtener velocidad angular actual del chasis
        ChassisSpeeds speeds = Constants.Swerve.kSwerveKinematics.toChassisSpeeds(getModuleStates());
        double velocidadAngular = Math.abs(speeds.omegaRadiansPerSecond);

        //Definir el umbral (Ajustable: 7.0 rad/s es un buen punto de partida)
        double maxVelocidadAngular = 7.0; 

        // Lógica de rechazo
        if (velocidadAngular > maxVelocidadAngular) {
            //Medición rechazada
            Logger.recordOutput("Vision/RejectedByVelocity", true);
            Logger.recordOutput("Vision/RejectionReason", "Giro demasiado rápido: " + velocidadAngular);
            return; // Salimos sin aplicar la medición
        }

        //Medición segura
        Logger.recordOutput("Vision/RejectedByVelocity", false);
        swerveOdometer.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurementStdDevs);
    }

    public Command sacudirChasis() {
        return this.run(() -> {
            // Obtenemos el tiempo actual del reloj de la roboRIO
            double tiempo = Timer.getFPGATimestamp();
            double frecuencia = SmartDashboard.getNumber("Meneito/Frecuencia", 10.0);
            double amplitud = SmartDashboard.getNumber("Meneito/Amplitud", 1.0);
            
            // Multiplicar el tiempo (ej. * 40) cambia qué tan rápido vibra (Frecuencia).
            // Multiplicar por fuera (ej. * 4.0) cambia qué tan fuerte gira (Amplitud en rad/s).
            double velocidadVibracion = Math.sin(tiempo * frecuencia) * amplitud; 
            
            // Le mandamos 0 en X y Y, y la vibración directa a la rotación (Robot Céntrico)
            drive(new Translation2d(0, 0), velocidadVibracion, false, true);
        })
        // Cuando suelten el botón, detenemos los motores de inmediato por seguridad
        .finallyDo(() -> stop()); 
    }

    public void simulationPeriodic() {
        // 1. Correr la física de cada módulo
        for (SwerveModule mod : swerveMods) {
            // Hacemos cast porque la interfaz SwerveModule no tiene este método
            if(mod instanceof RevSwerveModule) {
                ((RevSwerveModule)mod).simulationPeriodic(0.02);
            }
        }

        // 2. Calcular cómo se movió el chasis REALMENTE basado en las ruedas
        // Usamos los estados ACTUALES (simulados), no los deseados.
        SwerveModuleState[] realStates = getModuleStates();
        ChassisSpeeds actualSpeeds = Constants.Swerve.kSwerveKinematics.toChassisSpeeds(realStates);

        // 3. Actualizar el Giroscopio simulado
        // Ahora el gyro gira porque las ruedas giraron al chasis (¡Física!)
        // Multiplicamos por 1000ms/20ms = 50 o simplemente pasamos radianes/segundo si tu updateSimAngle lo soporta.
        // Viendo tu código anterior, updateSimAngle recibe (dt, radsPerSec):
        gyro.updateSimAngle(0.02, actualSpeeds.omegaRadiansPerSecond);
    }

  @Override
    public void periodic() { 
        // 1. Actualización de PID dinámico desde SmartDashboard
        double currentKP = SmartDashboard.getNumber("Tuning/Drive kP", Constants.Swerve.Drive.kP);
        double currentKD = SmartDashboard.getNumber("Tuning/Drive kD", Constants.Swerve.Drive.kD);

        if (currentKP != lastDriveKP || currentKD != lastDriveKD) {
            lastDriveKP = currentKP;
            lastDriveKD = currentKD;
            for (SwerveModule mod : swerveMods) {
                ((RevSwerveModule) mod).updateDrivePID(currentKP, currentKD);
            }
        }

        // 2. ACTUALIZACIÓN ÚNICA DE ODOMETRÍA (IMPORTANTE)
        swerveOdometer.update(getYaw(), getModulePositions());

        // 3. TELEMETRÍA Y LOGS
        Logger.recordOutput("Odometry/RobotPose", swerveOdometer.getEstimatedPosition());
        Logger.recordOutput("Swerve/ModuleStates/Real", getModuleStates());

        // Corregido: Obtener estados deseados correctamente
        SwerveModuleState[] desiredStates = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            desiredStates[mod.getModuleNumber()] = mod.getDesiredState();
        } // <--- Aquí faltaba cerrar este for correctamente
        
        Logger.recordOutput("Swerve/ModuleStates/Desired", desiredStates);

        // Field2d para el simulador y Dashboard
        field.setRobotPose(getPose());
        SmartDashboard.putData("field", field);
    } // Cierre de periodic
}