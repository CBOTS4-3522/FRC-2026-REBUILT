/*
 * Constants.java
 * 
 * Clase global estática que centraliza todas las constantes físicas, mapeo de hardware (IDs), 
 * ganancias de control (PID/FF) y configuraciones geométricas del robot.
 * El uso de clases anidadas (nested classes) ayuda a organizar las constantes por subsistema,
 * evitando la colisión de nombres y mejorando la legibilidad.
 */

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.lib.util.swerveUtil.SwerveConstants;

public final class Constants {

    // ==============================================================================
    // SUBSISTEMA: VISIÓN Y ODOMETRÍA
    // ==============================================================================
    public static final class VisionConstants {
        public static final String kCameraName = "atras";
        
        /** 
         * Transformación 3D desde el centro del robot hasta la lente de la cámara.
         * Fundamental para que PhotonVision/WPILib pueda proyectar la pose de la cámara al chasis.
         * Unidades en metros y radianes (Pitch de 50 grados, Yaw de 180).
         */
        public static final Transform3d kCameraOffset = new Transform3d(
                new Translation3d(((-(Constants.Swerve.kWheelBase) / 2) + 0.234),
                        ((-(Constants.Swerve.kTrackWidth) / 2) + 0.115), 0.403),
                new Rotation3d(0.0, Units.degreesToRadians(50), Units.degreesToRadians(180)) 
        );
        
        // Mapeo oficial de AprilTags en la cancha.
        public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltAndymark);
    }

    // ==============================================================================
    // CONFIGURACIÓN DE OPERADOR (INPUTS)
    // ==============================================================================
    public static final class OIConstants {
        // Zona muerta (Deadband) para ignorar el "drift" físico de los joysticks.
        public static final double kStickDeadband = 0.08;
        public static final int kDriver1Port = 0;
        public static final int kDriver2Port = 1;
    }

    // ==============================================================================
    // SUBSISTEMA: SHOOTER (Balística y Torreta)
    // ==============================================================================
    public static final class shooter {

        /**
         * Cálculo del Vector de Traslación del Shooter respecto al centro del chasis.
         * WPILib utiliza un sistema de coordenadas donde +X es el Frente y +Y es la Izquierda.
         * Se parte desde el Módulo 3 (Trasero-Izquierdo) y se aplica el offset medido en CAD (Onshape).
         */
        public static final Translation2d kOffsetDesdeModulo = new Translation2d(
                Units.inchesToMeters(3.663), 
                Units.inchesToMeters(-4.564) 
        );

        public static final Translation2d kShooterOffset = new Translation2d(
                -Constants.Swerve.kWheelBase / 2.0,
                Constants.Swerve.kTrackWidth / 2.0).plus(kOffsetDesdeModulo);

        // Coordenadas absolutas del objetivo ("Núcleo") en el mapa del campo (metros).
        public static final Translation2d kPosicionNucleoAzul = new Translation2d(4.625, 4.034);
        public static final Translation2d kPosicionNucleoRojo = new Translation2d(11.9148, 4.034);

        /**
         * Retorna la coordenada del objetivo basada en la alianza actual asignada por el FMS.
         */
        public static Translation2d getObjetivoActual() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return kPosicionNucleoRojo;
            }
            return kPosicionNucleoAzul; 
        }

        public static final class flywheels {
            public static final byte kLiderID = 50;
            public static final byte kSeguidorID = 51;
            
            // Lazo de control Cerrado (Feedback - PID)
            public static final double kP = 0.0001;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            // Constantes de FeedForward (Control Predictivo de Sistema) derivadas de SysId
            public static final double kS = 0.26395; // Fricción estática (Voltaje mínimo para mover)
            public static final double kV = 0.13;    // Constante de velocidad
            public static final double kA = 0.041581;// Constante de aceleración

            public static final double relationMotor = 4.0 / 3.0; // Relación de engranes
            public static final double defaultRPM = 3000;
        }

        public static final class azimuth {
            public static final byte kID = 54;
            
            public static final double kP = 0.22;
            public static final double kI = 0;
            public static final double kD = 0.00;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double kS = 0.15;

            // Restricciones cinemáticas para el ProfiledPIDController (Evita saltos de dientes y daños mecánicos)
            public static final double kMaxVelocityDegPerSec = 600.0;
            public static final double kMaxAccelerationDegPerSecSq = 800.0;
        }
    }

    // ==============================================================================
    // SUBSISTEMA: INDEXER (Manejo de carga)
    // ==============================================================================
    public static final class Indexer {
        public static final int kMotorBandasID = 53;
        public static final int kMotorMecanumID = 55;
        
        // Ciclos de trabajo PWM/CAN (Duty Cycle - 0.0 a 1.0)
        public static final double kVelocidadBandas = 0.5;
        public static final double kVelocidadMecanum = 1;
    }

    // ==============================================================================
    // SUBSISTEMA: INTAKE (Recolección)
    // ==============================================================================
    public static final class Intake {
        public static final int kMotorRollerID = 60;
        public static final int kMotorLiftID = 61;
        public static final byte kCanalEncoder = 0;

        // Limites lógicos y constantes de control del brazo actuador
        public static final double kEncoderOffset = 109.8;
        public static final double kTargetUp = 95;
        public static final double kTargetDown = 1;
        public static final double kTolerancyDegrees = 5.0;
        
        // Ganancias de PID (kG es la ganancia de gravedad para mantener el brazo estático contra su propio peso)
        public static final double kP = 0.01;
        public static final double kG = 0.8;
        public static final double kI = 0.0;
        public static final double kD = 0;

        // Umbral de amperaje para detección de colisión/final de carrera (Stall detection)
        public static final double kUmbralCorriente = 15.0;
    }

    // ==============================================================================
    // SUBSISTEMA: SWERVE DRIVE (Tracción Holonómica)
    // ==============================================================================
    public static final class Swerve {
        // Límite de aceleración para evitar vuelcos y deslizamientos (Slew Rate Limit)
        public static final double kSlewRateLimit = 3.0; 
        public static final boolean kIsOpenLoopTeleopSwerve = false;

        // Comportamiento de motores en estado inactivo (Brake frena en seco, Coast deja rodar)
        public static final SparkMaxConfig.IdleMode kDriveIdleMode = SparkMaxConfig.IdleMode.kBrake;
        public static final SparkMaxConfig.IdleMode kAngleIdleMode = SparkMaxConfig.IdleMode.kBrake;
        public static final SparkMaxConfig.IdleMode kIntakeIdleMode = SparkMaxConfig.IdleMode.kBrake;

        public static final SparkMaxConfig kDriveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig kAngleConfig = new SparkMaxConfig();

        public static final double kDrivePower = 1;
        public static final double kAnglePower = 1;

        public static final boolean kInvertGyro = false;

        // Configuración de Hardware: Módulos SDS MK4i con relación de engranajes L2
        public static final SwerveConstants kChosenModule = SwerveConstants
                .SDSMK4i(SwerveConstants.driveGearRatios.SDSMK4i_L2);

        // Mapeo de inversiones de polaridad basado en el módulo elegido
        public static final boolean kCanCoderInvert = kChosenModule.canCoderInvert;
        public static final boolean kDriveMotorInvert = kChosenModule.driveMotorInvert;
        public static final boolean kAngleMotorInvert = kChosenModule.angleMotorInvert;
        
        public static final double kAngleGearRatio = kChosenModule.angleGearRatio;
        public static final double kDriveGearRatio = kChosenModule.driveGearRatio;

        // Factores de conversión matemática (Rotaciones de motor -> Grados/Metros del sistema internacional)
        public static final double kDegreesPerTurnRotation = 360 / kAngleGearRatio;
        public static final double kDriveEncoderFactor = .957; // Ajuste empírico por fricción/compresión de banda
        public static final double kWheelCircumference = kChosenModule.wheelCircumference;
        public static final double kDriveRevToMeters = kWheelCircumference * kDriveEncoderFactor / (kDriveGearRatio);
        public static final double kDriveRpmToMetersPerSecond = kDriveRevToMeters / 60;

        /**
         * Dimensiones del Chasis.
         * TrackWidth: Distancia entre llantas izquierda-derecha.
         * WheelBase: Distancia entre llantas frontales-traseras.
         */
        public static final double kTrackWidth = Units.inchesToMeters(21.8503); 
        public static final double kWheelBase = Units.inchesToMeters(22);

        /**
         * Matriz Cinemática del Swerve.
         * Define las ubicaciones de los 4 módulos respecto al centro del robot.
         * Usada para transformar velocidades (m/s, rad/s) en estados individuales para cada módulo (velocidad y ángulo).
         */
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

        // Límites de Corriente (Current Limits) críticos para proteger el bus CAN y prevenir Brownouts de la PDP/PDH.
        public static final int kAngleContinuousCurrentLimit = 27;
        public static final int kAnglePeakCurrentLimit = 40;
        public static final double kAnglePeakCurrentDuration = 0.1;
        public static final boolean kAngleEnableCurrentLimit = true;
        
        public static final int kDriveContinuousCurrentLimit = 27;
        public static final int kDrivePeakCurrentLimit = 60;
        public static final double kDrivePeakCurrentDuration = 0.1;
        public static final boolean kDriveEnableCurrentLimit = true;

        public static final class Angle {
            public static final double kP = 0.01;
            public static final double kI = 0;
            public static final double kD = 0.00125;
            public static final double kS = 0.2;
            public static final double kV = 0.1;
            public static final double kA = 0.0;
        }

        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference) / kDriveGearRatio;

        public static final class Drive {
            public static final double kP = 0.0252;
            public static final double kI = 0.000031;
            public static final double kD = 0.01;

            // FeedForward para la tracción (Generado usualmente con herramienta SysId)
            public static final double kS = 0.05058075; // Volts requeridos para superar fricción
            public static final double kV = 2.695925;   // Volts requeridos para mantener 1 m/s
            public static final double kA = 0.9084375;  // Volts requeridos para acelerar 1 m/s^2
        }

        // Límites cinemáticos máximos físicos del chasis
        public static final double kMaxSpeed = 3.658; // m/s
        public static final double kMaxAngularVelocity = 7.0; // rad/s
        public static double kAngleRampRate = 0;

        // ==============================================================================
        // Mapeo Físico de Módulos (IDs y Offsets de CANCoder Absolutos)
        // ==============================================================================
        
        // Front Left - Módulo 0
        public static final class Mod0 {
            public static final int kDriveMotorId = 11;
            public static final int kAngleMotorId = 12;
            public static final int kCanCoderId = 13;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(56.88);
            public static final RevSwerveModuleConstants kConstants = new RevSwerveModuleConstants(kDriveMotorId, kAngleMotorId, kCanCoderId, kAngleOffset);
        }

        // Front Right - Módulo 1
        public static final class Mod1 {
            public static final int kDriveMotorId = 21;
            public static final int kAngleMotorId = 22;
            public static final int kCanCoderId = 23;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(248.4);
            public static final RevSwerveModuleConstants kConstants = new RevSwerveModuleConstants(kDriveMotorId, kAngleMotorId, kCanCoderId, kAngleOffset);
        }

        // Back Left - Módulo 2
        public static final class Mod2 {
            public static final int kDriveMotorId = 31;
            public static final int kAngleMotorId = 32;
            public static final int kCanCoderId = 33;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(101.52);
            public static final RevSwerveModuleConstants kConstants = new RevSwerveModuleConstants(kDriveMotorId, kAngleMotorId, kCanCoderId, kAngleOffset);
        }

        // Back Right - Módulo 3
        public static final class Mod3 {
            public static final int kDriveMotorId = 41;
            public static final int kAngleMotorId = 42;
            public static final int kCanCoderId = 43;
            public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(0.36);
            public static final RevSwerveModuleConstants kConstants = new RevSwerveModuleConstants(kDriveMotorId, kAngleMotorId, kCanCoderId, kAngleOffset);
        }
    }

    // ==============================================================================
    // CONFIGURACIÓN DE TRAYECTORIAS AUTÓNOMAS (PathPlanner)
    // ==============================================================================
    public static final class AutoConstants {
        // Límites para seguimiento de paths holonómicos
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

        // Restricciones de perfil trapezoidal para giros (Aceleración y desaceleración controladas)
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        // Velocidad en vacío del motor REV NEO 1.1 (Requerida para cálculos de cinemática)
        public static final double kFreeSpeedRpm = 5676;
    }
}