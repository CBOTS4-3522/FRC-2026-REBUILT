package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.Constants;
import frc.robot.subsystems.NavXGyro;

public class SwerveBase extends SubsystemBase {

    private double lastDriveKP = Constants.Swerve.Drive.kP;
    private double lastDriveKD = Constants.Swerve.Drive.kD;

    public SwerveDrivePoseEstimator swerveOdometer;
    public SwerveModule[] swerveMods;
    public NavXGyro gyro = NavXGyro.getInstance();

    // 1. EL PID COMPARTIDO PARA EL CHASIS
    public final PIDController headingController = new PIDController(0.022, 0.0, 0.0);

    private final Field2d field = new Field2d();
    private RobotConfig config;

    public SwerveBase() {
        headingController.enableContinuousInput(-180, 180);
        SmartDashboard.putData("Swerve/HeadingPID", headingController);

        SmartDashboard.putNumber("Turning/Drive kP", Constants.Swerve.Drive.kP);
        SmartDashboard.putNumber("Tuning/Drive kD", Constants.Swerve.Drive.kD);

        SmartDashboard.putNumber("SysId/Tiempo Quasistatic", 7.0);
        SmartDashboard.putNumber("SysId/Tiempo Dynamic", 1.5);

        SmartDashboard.putNumber("Meneito/Amplitud", 1.0);
        SmartDashboard.putNumber("Meneito/Frecuencia", 15);

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
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        // ==========================================================
        // WIDGET CUSTOMIZADO PARA ELASTIC (Swerve Drive)
        // ==========================================================
        SmartDashboard.putData("Elastic/Swerve Nativo", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                // Módulo 0 (Front Left)
                builder.addDoubleProperty("Front Left Angle", () -> swerveMods[0].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> swerveMods[0].getState().speedMetersPerSecond,
                        null);

                // Módulo 1 (Front Right)
                builder.addDoubleProperty("Front Right Angle", () -> swerveMods[1].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> swerveMods[1].getState().speedMetersPerSecond,
                        null);

                // Módulo 2 (Back Left)
                builder.addDoubleProperty("Back Left Angle", () -> swerveMods[2].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> swerveMods[2].getState().speedMetersPerSecond,
                        null);

                // Módulo 3 (Back Right)
                builder.addDoubleProperty("Back Right Angle", () -> swerveMods[3].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> swerveMods[3].getState().speedMetersPerSecond,
                        null);

                // Ángulo general del Robot
                builder.addDoubleProperty("Robot Angle", () -> getYaw().getRadians(), null);
            }
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        Rotation2d currentYaw = getYaw();
        ChassisSpeeds desiredChassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, currentYaw)
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

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

    public void wheelsIn() {
        swerveMods[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false, true);
        swerveMods[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)), false, true);
        swerveMods[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false, true);
        swerveMods[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-135)), false, true);
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
        return new ChassisSpeeds(twistForPose.dx / LOOP_TIME_S, twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
    }

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, Seconds.of(30.0), null),
            new SysIdRoutine.Mechanism(
                    (voltage) -> {
                        for (SwerveModule mod : swerveMods) {
                            mod.setDriveVoltage(voltage.in(Volts));
                            mod.lockAngle();
                        }
                    },
                    log -> {
                        for (SwerveModule mod : swerveMods) {
                            log.motor("drive-" + mod.getModuleNumber())
                                    .voltage(Volts.of(mod.getDriveVoltage()))
                                    .linearPosition(Meters.of(mod.getPosition().distanceMeters))
                                    .linearVelocity(MetersPerSecond.of(mod.getState().speedMetersPerSecond));
                        }
                    }, this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction)
                .withTimeout(Seconds.of(SmartDashboard.getNumber("SysId/Tiempo Quasistatic", 7.0)));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction)
                .withTimeout(Seconds.of(SmartDashboard.getNumber("SysId/Tiempo Dynamic", 1.5)));
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        // Ahora le pasamos los 3 parámetros al odómetro (o PoseEstimator)
        swerveOdometer.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    public Command sacudirChasis() {
        return this.run(() -> {
            double tiempo = Timer.getFPGATimestamp();
            double frecuencia = SmartDashboard.getNumber("Meneito/Frecuencia", 10.0);
            double amplitud = SmartDashboard.getNumber("Meneito/Amplitud", 1.0);
            double velocidadVibracion = Math.sin(tiempo * frecuencia) * amplitud;
            drive(new Translation2d(0, 0), velocidadVibracion, false, true);
        }).finallyDo(() -> stop());
    }

    public void simulationPeriodic() {
        for (SwerveModule mod : swerveMods) {
            if (mod instanceof RevSwerveModule) {
                ((RevSwerveModule) mod).simulationPeriodic(0.02);
            }
        }
        SwerveModuleState[] realStates = getModuleStates();
        ChassisSpeeds actualSpeeds = Constants.Swerve.kSwerveKinematics.toChassisSpeeds(realStates);
        gyro.updateSimAngle(0.02, actualSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void periodic() {
        double currentKP = SmartDashboard.getNumber("Tuning/Drive kP", Constants.Swerve.Drive.kP);
        double currentKD = SmartDashboard.getNumber("Tuning/Drive kD", Constants.Swerve.Drive.kD);

        if (currentKP != lastDriveKP || currentKD != lastDriveKD) {
            lastDriveKP = currentKP;
            lastDriveKD = currentKD;
            for (SwerveModule mod : swerveMods) {
                ((RevSwerveModule) mod).updateDrivePID(currentKP, currentKD);
            }
        }

        swerveOdometer.update(getYaw(), getModulePositions());
        Logger.recordOutput("Odometry/RobotPose", swerveOdometer.getEstimatedPosition());
        Logger.recordOutput("Swerve/ModuleStates/Real", getModuleStates());

        SwerveModuleState[] desiredStates = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            desiredStates[mod.getModuleNumber()] = mod.getDesiredState();
        } // <---- ¡ESTA ES LA LLAVE QUE FALTABA!

        Logger.recordOutput("Swerve/ModuleStates/Desired", desiredStates);

        SmartDashboard.putData("field", field);
        field.setRobotPose(getPose());

        // ==========================================================
        // TELEMETRÍA PARA EL WIDGET SWERVE DE ELASTIC
        // ==========================================================
        SwerveModuleState[] estados = getModuleStates();
        double[] datosSwerveElastic = new double[] {
                // Módulo 0 (Front Left)
                estados[0].angle.getDegrees(), estados[0].speedMetersPerSecond,
                // Módulo 1 (Front Right)
                estados[1].angle.getDegrees(), estados[1].speedMetersPerSecond,
                // Módulo 2 (Back Left)
                estados[2].angle.getDegrees(), estados[2].speedMetersPerSecond,
                // Módulo 3 (Back Right)
                estados[3].angle.getDegrees(), estados[3].speedMetersPerSecond
        };
        SmartDashboard.putNumberArray("Elastic/EstadosSwerve", datosSwerveElastic);

        // Calculamos la velocidad real del robot combinando X y Y usando Pitágoras
        double velocidadTotal = Math.hypot(getRobotRelativeSpeeds().vxMetersPerSecond,
                getRobotRelativeSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Elastic/Velocidad", velocidadTotal);
    }
}