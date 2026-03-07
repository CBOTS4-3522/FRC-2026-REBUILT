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
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.lib.util.swerveUtil.SwerveConstants;


public final class Constants {

	public static final class VisionConstants {
		public static final String kCameraName = "CAM_POSE"; // Cambia esto al nombre de tu cámara
		public static final Transform3d kCameraOffset = new Transform3d(
            new Translation3d(0.0, 0.0, Units.inchesToMeters(46.0)), // X, Y, Z (46 pulgadas de alto)
            new Rotation3d(0.0, Units.degreesToRadians(-60.0), 0.0)  // Roll, Pitch, Yaw
        );
		public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	}


	public static final class OIConstants {
		public static final double kStickDeadband = 0.08;
		public static final int kDriver1Port = 0;
		public static final int kDriver2Port = 1;
	}

	public static final class shooter {

		// ... dentro de public static final class shooter { ...

		// 1. El pequeño offset medido en Onshape (Desde el Módulo 3 hacia el Shooter)
		// NOTA DE SIGNOS WPILib: +X es hacia el FRENTE del robot, +Y es hacia la
		// IZQUIERDA.
		// Si el shooter está "hacia el frente" y "hacia el centro (derecha)" respecto a
		// la llanta trasera-izquierda:
		public static final Translation2d kOffsetDesdeModulo = new Translation2d(
				Units.inchesToMeters(3.663), // X: Distancia hacia el frente
				Units.inchesToMeters(-4.564) // Y: Distancia hacia el centro (Derecha es negativo en WPILib)
		);

		// 2. El Offset Total (Centro del Robot -> Shooter)
		// Literalmente le decimos: "Ve a la llanta trasera-izquierda, y súmale el
		// offset de Onshape"
		public static final Translation2d kShooterOffset = new Translation2d(
				-Constants.Swerve.kWheelBase / 2.0,
				Constants.Swerve.kTrackWidth / 2.0).plus(kOffsetDesdeModulo);

		// Coordenada exacta del Núcleo (Juego: Rebuilt)
		public static final Translation2d kPosicionMeta = new Translation2d(4.625, 4.034);

		public static final class flywheels {
			public static final byte kLiderID = 50;
			public static final byte kSeguidorID = 51;
			public static final double kP = 0.0003;
			public static final double kI = 0.000000022;
			public static final double kD = 0.001;

			public static final double kS = 0.25941;
			public static final double kV = 0.125;
			public static final double kA = 0.040438;

			public static final double relationMotor = 4.0 / 3.0;

			public static final double defaultRPM = 3000;

		}

		public static final class azimuth {
			public static final byte kID = 54;
			public static final double kP = 0.5;
			public static final double kI = 0;
			public static final double kD = 0.01;

			public static final double kV = 0.0;
			public static final double kA = 0.0;
			public static final double kS = 0.0;
		}
	}

	public static final class Indexer {
		public static final int kMotorBandasID = 53;
		public static final int kMotorMecanumID = 55;
		public static final double kVelocidadBandas = 1;
		public static final double kVelocidadMecanum = 0.5;

	}

	public static final class Intake {

		// IDs
		public static final int kMotorRollerID = 60;
		public static final int kMotorLiftID = 61;

		// Canales
		public static final byte kCanalEncoder = 0;

		// Logica del brazo
		public static final double kEncoderOffset = 109.8;
		public static final double kTargetUp = 95;
		public static final double kTargetDown = 1;
		public static final double kTolerancyDegrees = 5.0;
		public static final double kP = 0.01;
		public static final double kG = 0.8;
		public static final double kI = 0.0;
		public static final double kD = 0;

		// Contador
		public static final double kUmbralCorriente = 15.0;

	}

	public static final class Swerve {

		public static final double kSlewRateLimit = 3.0; // Unidades por segundo (de 0 a 1)
		public static final boolean kIsOpenLoopTeleopSwerve = false;

		// Spark Max Idle Modes
		public static final SparkMaxConfig.IdleMode kDriveIdleMode = SparkMaxConfig.IdleMode.kBrake;
		public static final SparkMaxConfig.IdleMode kAngleIdleMode = SparkMaxConfig.IdleMode.kBrake;

		public static final SparkMaxConfig.IdleMode kIntakeIdleMode = SparkMaxConfig.IdleMode.kBrake;

		// Spark Max Configs
		public static final SparkMaxConfig kDriveConfig = new SparkMaxConfig();
		public static final SparkMaxConfig kAngleConfig = new SparkMaxConfig();

		// Max Output Powers
		public static final double kDrivePower = 1;
		public static final double kAnglePower = 1;

		// Gyro
		public static final boolean kInvertGyro = false;

		// Swerve Module Type
		public static final SwerveConstants kChosenModule = SwerveConstants
				.SDSMK4i(SwerveConstants.driveGearRatios.SDSMK4i_L2);

		/* Angle Encoder Invert */
		public static final boolean kCanCoderInvert = kChosenModule.canCoderInvert;
		public static final boolean kDriveMotorInvert = kChosenModule.driveMotorInvert;

		/* Motor Inverts */
		public static final boolean kAngleMotorInvert = kChosenModule.angleMotorInvert;
		public static final double kAngleGearRatio = kChosenModule.angleGearRatio;

		// Ratios
		public static final double kDegreesPerTurnRotation = 360 / kAngleGearRatio;
		public static final double kDriveGearRatio = kChosenModule.driveGearRatio;

		// meters per rotation
		public static final double kDriveEncoderFactor = .957;
		public static final double kWheelCircumference = kChosenModule.wheelCircumference;
		public static final double kDriveRevToMeters = kWheelCircumference * kDriveEncoderFactor / (kDriveGearRatio);
		public static final double kDriveRpmToMetersPerSecond = kDriveRevToMeters / 60;

		/* Drivetrain Constants */
		public static final double kTrackWidth = Units.inchesToMeters(21.8503); // 55.5cm 0.555 M
		public static final double kWheelBase = Units.inchesToMeters(22);// 55.88 cm 0.5588M

		// Swerve Kinematics
		public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

		/* Swerve Current Limiting */
		public static final int kAngleContinuousCurrentLimit = 27;
		public static final int kAnglePeakCurrentLimit = 40;
		public static final double kAnglePeakCurrentDuration = 0.1;
		public static final boolean kAngleEnableCurrentLimit = true;
		public static final int kDriveContinuousCurrentLimit = 27;
		public static final int kDrivePeakCurrentLimit = 60;
		public static final double kDrivePeakCurrentDuration = 0.1;
		public static final boolean kDriveEnableCurrentLimit = true;

		public static final class Angle {
			/* Angle Motor PID Values */
			public static final double kP = 0.01;
			public static final double kI = 0;
			public static final double kD = 0.00125;

			/* Angle motor FeedForward Values */
			public static final double kS = 0.2;
			public static final double kV = 0.1;
			public static final double kA = 0.0;
		}

		/* Drive Motor info */
		public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference)
				/ kDriveGearRatio;

		public static final class Drive {
			public static final double kP = 0.02;
			public static final double kI = 0.0;
			public static final double kD = 0.01;

			/* Drive motor FeedForward Values */
			public static final double kS = 0.1615675; // Voltios para empezar a mover
			public static final double kV = 2.56015; // Voltios por metro/segundo
			public static final double kA = 0.3928225; // Voltios por metro/segundo^2

		}

		/** Meters per Second */
		public static final double kMaxSpeed = 3.658;

		/** Radians per Second */
		public static final double kMaxAngularVelocity = 7.0;
		public static double kAngleRampRate = 0;

		////////////// Swerve Module Constants////////////

		/* Front Left Module - Module 1 */
		public static final class Mod0 {
			public static final int kDriveMotorId = 11;
			public static final int kAngleMotorId = 12;
			public static final int kCanCoderId = 13;
			public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(56.88);
			public static final RevSwerveModuleConstants kConstants = new RevSwerveModuleConstants(kDriveMotorId,
					kAngleMotorId, kCanCoderId, kAngleOffset);
		}

		/* Front Right Module - Module 2 */
		public static final class Mod1 {
			public static final int kDriveMotorId = 21;
			public static final int kAngleMotorId = 22;
			public static final int kCanCoderId = 23;
			public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(248.4);
			public static final RevSwerveModuleConstants kConstants = new RevSwerveModuleConstants(kDriveMotorId,
					kAngleMotorId, kCanCoderId, kAngleOffset);
		}

		/* Back Left Module - Module 3 */
		public static final class Mod2 {
			public static final int kDriveMotorId = 31;
			public static final int kAngleMotorId = 32;
			public static final int kCanCoderId = 33;
			public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(101.52);
			public static final RevSwerveModuleConstants kConstants = new RevSwerveModuleConstants(kDriveMotorId,
					kAngleMotorId, kCanCoderId, kAngleOffset);
		}

		/* Back Right Module - Module 4 */
		public static final class Mod3 {
			public static final int kDriveMotorId = 41;
			public static final int kAngleMotorId = 42;
			public static final int kCanCoderId = 43;
			public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(0.36);
			public static final RevSwerveModuleConstants kConstants = new RevSwerveModuleConstants(kDriveMotorId,
					kAngleMotorId, kCanCoderId, kAngleOffset);
		}
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 2;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

		// Motion profilied robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond,
				kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}

}