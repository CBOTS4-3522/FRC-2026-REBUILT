package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.swerveUtil.CTREState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class RevSwerveModule implements SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;

    // motores de simulacion
    private DCMotorSim mDriveSim;
    private DCMotorSim mAngleSim;

    // Objetos de configuración (REV 2025)
    private SparkMaxConfig mAngleConfig;
    private SparkMaxConfig mDriveConfig;

    private CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    public SwerveModuleState desiredState;
    public final FeedForwardConfig feedForward = new FeedForwardConfig();

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;

        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Motor Config */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleConfig = new SparkMaxConfig();
        configAngleMotor(); // Prepara el config, no lo aplica todavía

        /* Drive Motor Config */
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        mDriveConfig = new SparkMaxConfig();
        configDriveMotor(); // Prepara el config, no lo aplica todavía

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);

        // Configura encoders y APLICA la configuración a los motores
        configEncoders();

        // Sincronizar encoders (Absolute -> Relative)
        synchronizeEncoders();
        this.desiredState = new SwerveModuleState(0, getAngle());

        // ... dentro del constructor de RevSwerveModule ...

        // 1. Definimos la física del sistema (La "Planta")
        // createDCMotorSystem(Motor, Inercia J [kg*m^2], Relación de Engranes)

        // Para el Drive (Rueda)
        var drivePlant = LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), // Motor
                0.025, // Inercia aprox (J)
                Constants.Swerve.kDriveGearRatio // Gearing
        );

        // Para el Angle (Dirección)
        var anglePlant = LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), // Motor
                0.004, // Inercia aprox (J)
                Constants.Swerve.kAngleGearRatio // Gearing
        );

        // 2. Creamos la simulación usando la planta
        // El segundo parámetro es el gearbox, que usamos para saber limites de
        // corriente, etc.
        mDriveSim = new DCMotorSim(drivePlant, DCMotor.getNEO(1));
        mAngleSim = new DCMotorSim(anglePlant, DCMotor.getNEO(1));
    }

    private void configEncoders() {
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        // Factores de conversión
        mDriveConfig.encoder.positionConversionFactor(Constants.Swerve.kDriveRevToMeters);
        mDriveConfig.encoder.velocityConversionFactor(Constants.Swerve.kDriveRpmToMetersPerSecond);

        relAngleEncoder = mAngleMotor.getEncoder();
        mAngleConfig.encoder.positionConversionFactor(Constants.Swerve.kDegreesPerTurnRotation);
        mAngleConfig.encoder.velocityConversionFactor(Constants.Swerve.kDegreesPerTurnRotation / 60);

        mDriveMotor.configure(mDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        mAngleMotor.configure(mAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configAngleMotor() {
        mAngleConfig.closedLoop.p(Constants.Swerve.Angle.kP, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.i(Constants.Swerve.Angle.kI, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.d(Constants.Swerve.Angle.kD, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.feedForward.kS(Constants.Swerve.Angle.kS, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.feedForward.kV(Constants.Swerve.Angle.kV, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.feedForward.kA(Constants.Swerve.Angle.kA, ClosedLoopSlot.kSlot0);
        mAngleConfig.closedLoop.outputRange(-Constants.Swerve.kAnglePower, Constants.Swerve.kAnglePower);

        // CORRECCION: Usar el límite de corriente de ANGLE, no de DRIVE
        mAngleConfig.smartCurrentLimit(Constants.Swerve.kAngleContinuousCurrentLimit);

        mAngleConfig.inverted(Constants.Swerve.kAngleMotorInvert);
        mAngleConfig.idleMode(Constants.Swerve.kAngleIdleMode);
        mAngleConfig.closedLoopRampRate(Constants.Swerve.kAngleRampRate);
    }

    private void configDriveMotor() {
        // Aseguramos usar las constantes de DRIVE (KP, KI, KD)
        mDriveConfig.closedLoop.p(Constants.Swerve.Drive.kP, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.i(Constants.Swerve.Drive.kI, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.d(Constants.Swerve.Drive.kD, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.feedForward.kA(Constants.Swerve.Drive.kA, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.feedForward.kV(Constants.Swerve.Drive.kV, ClosedLoopSlot.kSlot0);
        mDriveConfig.closedLoop.feedForward.kS(Constants.Swerve.Drive.kS, ClosedLoopSlot.kSlot0);

        mDriveConfig.closedLoop.outputRange(-1, 1);
        mDriveConfig.smartCurrentLimit(Constants.Swerve.kDriveContinuousCurrentLimit);

        mDriveConfig.inverted(Constants.Swerve.kDriveMotorInvert);
        mDriveConfig.idleMode(Constants.Swerve.kDriveIdleMode);
    }

    @Override // Implementando de la interfaz
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Optimizar para no girar más de 90 grados
        this.desiredState = CTREState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        setSpeed(this.desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.kMaxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        double velocity = desiredState.speedMetersPerSecond;
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        controller.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevenir Jitter: Si la velocidad es muy baja, no muevas el ángulo
        if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.kMaxSpeed * 0.01)) {
            mAngleMotor.stopMotor();
            return;
        }

        Rotation2d angle = desiredState.angle;
        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();
        double degReference = angle.getDegrees();
        controller.setSetpoint(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    @Override
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    @Override
    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }

    @Override
    public void synchronizeEncoders() {
        // Alinea el encoder relativo del NEO con el absoluto del CANCoder
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }

    @Override
    public SwerveModuleState getState() {
        // Si estamos en simulación, leemos la velocidad directo de la física
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            double simVel = mDriveSim.getAngularVelocityRadPerSec() * (Constants.Swerve.kWheelCircumference / (2 * Math.PI));
            return new SwerveModuleState(simVel, getAngle());
        }
        
        // Si es real, usamos el encoder
        return new SwerveModuleState(relDriveEncoder.getVelocity(), getAngle());
    }

    @Override
    public double getOmega() {
        // Retorna la velocidad angular en grados por segundo (o la unidad que
        // necesites)
        // OJO: Checar si 'getVelocity' de CANCoder retorna Rotaciones/Seg o Grados/Seg
        // Usualmente Phoenix 6 retorna Rotaciones/Seg, por eso * 360 estaba en tu
        // logica anterior
        return angleEncoder.getVelocity().getValueAsDouble() * 360;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition(),
                getAngle());
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    // Dentro de RevSwerveModule.java
    public void setDriveVoltage(double volts) {
        mDriveMotor.setVoltage(volts);
    }

    @Override
    public double getDriveVoltage() {
        // El voltaje aplicado es el voltaje de la batería por el porcentaje de salida
        return mDriveMotor.getBusVoltage() * mDriveMotor.getAppliedOutput();
    }

    @Override
    public void lockAngle() {
        // Forzamos a que el módulo siempre mire hacia adelante (0 grados)
        mAngleMotor.getClosedLoopController().setSetpoint(0, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    /**
     * Este método corre cada 20ms SOLO en el simulador.
     */
    public void simulationPeriodic(double dt) {
        // --- 1. CÁLCULO MANUAL DE VOLTAJES ---
        // Como el SparkMax Sim no calcula el PID solo, lo hacemos nosotros con las constantes que ya tienes.
        
        // A) Voltaje de DRIVE (Feedforward + Friction)
        double velocitySetpoint = desiredState.speedMetersPerSecond;
        // Fórmula: Velocidad * kV + kS (para romper fricción)
        double driveVoltage = velocitySetpoint * Constants.Swerve.Drive.kV;
        if (Math.abs(velocitySetpoint) > 0.01) {
            driveVoltage += Math.signum(velocitySetpoint) * Constants.Swerve.Drive.kS;
        }

        // B) Voltaje de ANGLE (PID Simple)
        // Calculamos el error en grados (Rotation2d maneja la lógica de -180 a 180 automáticamente)
        double angleErrorDegrees = desiredState.angle.minus(getAngle()).getDegrees();
        // Fórmula: Error * kP * 12V (Multiplicamos por 12 porque kP es para output 0-1)
        double angleVoltage = angleErrorDegrees * Constants.Swerve.Angle.kP * 12.0;

        // Limitamos el voltaje a lo que da la batería (clamp)
        driveVoltage = MathUtil.clamp(driveVoltage, -12.0, 12.0);
        angleVoltage = MathUtil.clamp(angleVoltage, -12.0, 12.0);


        // --- 2. FÍSICA (DCMotorSim) ---
        mDriveSim.setInputVoltage(driveVoltage);
        mAngleSim.setInputVoltage(angleVoltage);

        mDriveSim.update(dt);
        mAngleSim.update(dt);


        // --- 3. ACTUALIZAR SENSORES ---
        
        // Drive Position (Radianes Sim -> Metros Encoder)
        double drivePosMeters = (mDriveSim.getAngularPositionRad() / (2 * Math.PI)) * Constants.Swerve.kWheelCircumference;
        relDriveEncoder.setPosition(drivePosMeters);

        // Angle Position (Radianes Sim -> Grados Encoder)
        
        double angleDegrees = Math.toDegrees(mAngleSim.getAngularPositionRad());
        
        relAngleEncoder.setPosition(angleDegrees);
    }

}