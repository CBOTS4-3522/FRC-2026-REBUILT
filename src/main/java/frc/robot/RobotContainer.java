package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.intake.IntakeLift;
import frc.robot.subsystems.intake.IntakeLiftIO;
import frc.robot.subsystems.intake.IntakeLiftOIReal;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeRollersIO;
import frc.robot.subsystems.intake.IntakeRollersIOReal;
import frc.robot.subsystems.shooter.ShooterAzimuth;
import frc.robot.subsystems.shooter.ShooterAzimuthIOReal;
import frc.robot.subsystems.shooter.ShooterAzimuthIO;
import frc.robot.subsystems.shooter.ShooterFlywheels;
import frc.robot.subsystems.shooter.ShooterFlywheelsIO;
import frc.robot.subsystems.shooter.ShooterFlywheelsIOReal;
import frc.robot.subsystems.swerve.SwerveBase;

public class RobotContainer {
        public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

        private final CommandXboxController driver1 = new CommandXboxController(Constants.OIConstants.kDriver1Port);
        private final CommandXboxController driver2 = new CommandXboxController(Constants.OIConstants.kDriver2Port);

        private final InterpolatingDoubleTreeMap mapaRPM = new InterpolatingDoubleTreeMap();
        private final InterpolatingDoubleTreeMap mapaTiempo = new InterpolatingDoubleTreeMap();
        private final InterpolatingDoubleTreeMap mapaChamfle = new InterpolatingDoubleTreeMap();

        private final SwerveBase s_Swerve;
        private final ShooterAzimuth s_ShooterAzimuth;
        private final ShooterFlywheels s_ShooterFlywheels;
        private final Indexer s_Indexer;
        private final IntakeLift s_IntakeLift;
        private final IntakeRollers s_IntakeRollers;
        private final LedSubsystem s_LedSubsystem;

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                s_Swerve = new SwerveBase();
                s_Indexer = new Indexer();
                s_LedSubsystem = new LedSubsystem();

                mapaRPM.put(1.5, 2600.0);
                mapaTiempo.put(1.5, 1.20);
                mapaChamfle.put(1.5, 0.0);
                mapaRPM.put(2.0, 2800.0);
                mapaTiempo.put(2.0, 1.00);
                mapaChamfle.put(2.0, 0.0);
                mapaRPM.put(2.5, 3000.0);
                mapaTiempo.put(2.5, 1.48);
                mapaChamfle.put(2.5, 60.0);
                mapaRPM.put(3.0, 3200.0);
                mapaTiempo.put(3.0, 1.35);
                mapaChamfle.put(3.0, 80.0);
                mapaRPM.put(3.5, 3400.0);
                mapaTiempo.put(3.5, 1.45);
                mapaChamfle.put(3.5, 100.0);
                mapaRPM.put(4.0, 3600.0);
                mapaTiempo.put(4.0, 1.58);
                mapaChamfle.put(4.0, 120.0);
                mapaRPM.put(4.5, 3800.0);
                mapaTiempo.put(4.5, 1.75);
                mapaChamfle.put(4.5, 140.0);

                if (Robot.isReal()) {
                        s_ShooterAzimuth = new ShooterAzimuth(new ShooterAzimuthIOReal());
                } else {
                        s_ShooterAzimuth = new ShooterAzimuth(new ShooterAzimuthIO() {
                        });
                }

                if (Robot.isReal()) {
                        s_ShooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIOReal());
                } else {
                        s_ShooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIO() {
                        });
                }

                IntakeLiftIO intakeLiftIO;
                if (RobotBase.isReal()) {
                        intakeLiftIO = new IntakeLiftOIReal();
                } else {
                        intakeLiftIO = new IntakeLiftIO() {
                        };
                }
                s_IntakeLift = new IntakeLift(intakeLiftIO);

                IntakeRollersIO intakeRollersIO;
                if (RobotBase.isReal()) {
                        intakeRollersIO = new IntakeRollersIOReal();
                } else {
                        intakeRollersIO = new IntakeRollersIO() {
                        };
                }
                s_IntakeRollers = new IntakeRollers(intakeRollersIO);

                ShuffleboardTab diagTab = Shuffleboard.getTab("Diagnóstico");

                diagTab.add("Quasistatic Forward", new DeferredCommand(
                                () -> s_Swerve.sysIdQuasistatic(Direction.kForward), Set.of(s_Swerve))).withSize(2, 1)
                                .withPosition(0, 0);
                diagTab.add("Quasistatic Reverse", new DeferredCommand(
                                () -> s_Swerve.sysIdQuasistatic(Direction.kReverse), Set.of(s_Swerve))).withSize(2, 1)
                                .withPosition(2, 0);
                diagTab.add("Dynamic Forward",
                                new DeferredCommand(() -> s_Swerve.sysIdDynamic(Direction.kForward), Set.of(s_Swerve)))
                                .withSize(2, 1).withPosition(0, 1);
                diagTab.add("Dynamic Reverse",
                                new DeferredCommand(() -> s_Swerve.sysIdDynamic(Direction.kReverse), Set.of(s_Swerve)))
                                .withSize(2, 1).withPosition(2, 1);
                diagTab.add("Gyro", s_Swerve.gyro).withWidget(BuiltInWidgets.kGyro).withSize(2, 2).withPosition(4, 0);

                diagTab.add("Shooter QS Fwd",
                                new DeferredCommand(() -> s_ShooterFlywheels.sysIdQuasistatic(Direction.kForward),
                                                Set.of(s_ShooterFlywheels)))
                                .withSize(2, 1).withPosition(0, 2);
                diagTab.add("Shooter QS Rev",
                                new DeferredCommand(() -> s_ShooterFlywheels.sysIdQuasistatic(Direction.kReverse),
                                                Set.of(s_ShooterFlywheels)))
                                .withSize(2, 1).withPosition(2, 2);
                diagTab.add("Shooter Dyn Fwd", new DeferredCommand(
                                () -> s_ShooterFlywheels.sysIdDynamic(Direction.kForward), Set.of(s_ShooterFlywheels)))
                                .withSize(2, 1).withPosition(0, 3);
                diagTab.add("Shooter Dyn Rev", new DeferredCommand(
                                () -> s_ShooterFlywheels.sysIdDynamic(Direction.kReverse), Set.of(s_ShooterFlywheels)))
                                .withSize(2, 1).withPosition(2, 3);

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Selector", autoChooser);

                s_Swerve.setDefaultCommand(new TeleopSwerve(
                                s_Swerve,
                                () -> -driver1.getLeftY(),
                                () -> -driver1.getLeftX(),
                                () -> driver1.getRightX(),
                                () -> driver1.getLeftTriggerAxis(),
                                () -> driver1.getHID().getLeftBumperButton(),
                                () -> driver1.getHID().getRightBumperButton(),
                                () -> driver1.getHID().getPOV()));

                if (RobotBase.isReal()) {
                        SmartDashboard.putData("Calibracion/Quasistatic Forward", new DeferredCommand(
                                        () -> s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                                        Set.of(s_Swerve)));
                        SmartDashboard.putData("Calibracion/Quasistatic Reverse", new DeferredCommand(
                                        () -> s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                                        Set.of(s_Swerve)));
                        SmartDashboard.putData("Calibracion/Dynamic Forward",
                                        new DeferredCommand(
                                                        () -> s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward),
                                                        Set.of(s_Swerve)));
                        SmartDashboard.putData("Calibracion/Dynamic Reverse",
                                        new DeferredCommand(
                                                        () -> s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse),
                                                        Set.of(s_Swerve)));
                }

                s_ShooterAzimuth.setDefaultCommand(s_ShooterAzimuth.controlManualAzimuth(
                                () -> driver2.getLeftX(),
                                () -> -driver2.getLeftY()));

                configureButtonBindings();
        }

        public Command dispararEnMovimientoChasis(DoubleSupplier translationX, DoubleSupplier translationY) {
                return Commands.run(() -> {
                        Pose2d robotPose = s_Swerve.getPose();
                        Rotation2d robotYaw = s_Swerve.getYaw();

                        double distanciaCentroMeta = robotPose.getTranslation()
                                        .getDistance(Constants.shooter.kPosicionMeta);
                        double tiempoVuelo = mapaTiempo.get(distanciaCentroMeta);

                        ChassisSpeeds velRobot = s_Swerve.getRobotRelativeSpeeds();
                        Translation2d vectorVelocidadCampo = new Translation2d(
                                        velRobot.vxMetersPerSecond, velRobot.vyMetersPerSecond).rotateBy(robotYaw);

                        Translation2d desviacionPorInercia = vectorVelocidadCampo.times(tiempoVuelo);
                        Translation2d metaVirtual = Constants.shooter.kPosicionMeta.minus(desviacionPorInercia);

                        double distanciaMetaVirtual = robotPose.getTranslation().getDistance(metaVirtual);
                        double offsetLateralY = Constants.shooter.kShooterOffset.getY();

                        Rotation2d anguloCentroHaciaMeta = Rotation2d.fromRadians(
                                        Math.atan2(
                                                        metaVirtual.getY() - robotPose.getY(),
                                                        metaVirtual.getX() - robotPose.getX()));

                        double compensacionParalajeRads = Math.asin(offsetLateralY / distanciaMetaVirtual);
                        Rotation2d anguloChasisDeseado = anguloCentroHaciaMeta
                                        .minus(Rotation2d.fromRadians(compensacionParalajeRads));

                        double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(),
                                        Constants.OIConstants.kStickDeadband);
                        double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(),
                                        Constants.OIConstants.kStickDeadband);

                        double rotacionPID = s_Swerve.headingController.calculate(robotYaw.getDegrees(),
                                        anguloChasisDeseado.getDegrees());
                        double rotacionLimitada = MathUtil.clamp(rotacionPID, -1.0, 1.0);

                        s_Swerve.drive(
                                        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.kMaxSpeed),
                                        rotacionLimitada * Constants.Swerve.kMaxAngularVelocity,
                                        true,
                                        false);

                        Translation2d offsetRotado = Constants.shooter.kShooterOffset.rotateBy(robotYaw);
                        Translation2d posicionShooterReal = robotPose.getTranslation().plus(offsetRotado);
                        double distanciaFinal = posicionShooterReal.getDistance(metaVirtual);

                        double rpmDeseado = mapaRPM.get(distanciaFinal);
                        double chamfleDeseado = mapaChamfle.get(distanciaFinal);

                        s_ShooterFlywheels.setObjetivoRPM(rpmDeseado);
                        s_ShooterAzimuth.setObjetivo(chamfleDeseado);

                        SmartDashboard.putNumber("AutoAim/AnguloChasis_Deseado", anguloChasisDeseado.getDegrees());

                }, s_Swerve, s_ShooterFlywheels, s_ShooterAzimuth)
                                .finallyDo(() -> {
                                        s_ShooterFlywheels.detener();
                                        s_Swerve.stop();
                                });
        }

        public Command vibrarDriver(CommandXboxController driverM, XboxController.RumbleType tipo, double magnitud,
                        double tiempo) {
                return Commands.runEnd(
                                () -> driverM.getHID().setRumble(tipo, magnitud),
                                () -> driverM.getHID().setRumble(tipo, 0)).withTimeout(tiempo);
        }

        private void configureButtonBindings() {
                driver1.rightStick().onTrue(new InstantCommand(s_Swerve::zeroGyro));
                driver1.start().onTrue(
                                Commands.runOnce(() -> {
                                        s_Swerve.resetOdometry(new Pose2d(3.571, 4.0, Rotation2d.fromDegrees(180)));
                                }));
                driver1.leftStick().whileTrue(s_Swerve.sacudirChasis());

                driver2.x().whileTrue(s_IntakeRollers.tragarPelotas());
                driver2.leftBumper().whileTrue(s_Indexer.encender());
                driver2.leftTrigger().whileTrue(s_Indexer.alRevez());
                driver2.a().onTrue(s_IntakeLift.bajarProtegido());
                driver2.b().onTrue(s_IntakeLift.subirProtegido());
                driver2.rightStick().whileTrue(s_ShooterFlywheels.testShooterDesdeDashboard());

                // Botón Supremo de Apuntar y Disparar usando el PID del Chasis
                driver2.y().whileTrue(
                                dispararEnMovimientoChasis(
                                                () -> -driver1.getLeftY(),
                                                () -> -driver1.getLeftX()).alongWith(
                                                                Commands.sequence(
                                                                                Commands.waitUntil(
                                                                                                s_ShooterFlywheels::estaEnVelocidad),
                                                                                s_Indexer.encender())));

                driver2.rightBumper().whileTrue(s_IntakeRollers.movimiento());
                
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}