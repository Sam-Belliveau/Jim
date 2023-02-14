package com.stuypulse.robot.subsystems.swerve.modules;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ExampleMAXSwerveModule extends SwerveModule {
    
    // module data
    private final String id;
    private final Translation2d offset;
    private final Rotation2d angleOffset;
    private SwerveModuleState targetState;

    // turn
    private final CANSparkMax turnMotor;
    private final AbsoluteEncoder turnEncoder;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    // controller
    private final SparkMaxPIDController drivePID;
    private final SparkMaxPIDController turnPID;
  
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public ExampleMAXSwerveModule(String id, Translation2d offset, int turningCANId, Rotation2d angleOffset, int drivingCANId) {
        this.id = id;
        this.offset = offset;
        this.angleOffset = angleOffset;
        targetState = new SwerveModuleState(0, new Rotation2d());

        driveMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        drivePID = driveMotor.getPIDController();
        turnPID = turnMotor.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);
        turnPID.setFeedbackDevice(turnEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turnEncoder.setPositionConversionFactor(2 * Math.PI);
        turnEncoder.setVelocityConversionFactor(2 * Math.PI / 60.0);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        turnEncoder.setInverted(false);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(Encoder.Turn.MIN_PID_INPUT);
        turnPID.setPositionPIDWrappingMaxInput(Encoder.Turn.MAX_PID_INPUT);
        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        drivePID.setP(Turn.kP);
        drivePID.setI(Turn.kI);
        drivePID.setD(Turn.kD);
        // m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        // m_drivingPIDController.setOutputRange(-1, 1);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        turnPID.setP(Drive.kP);
        turnPID.setI(Drive.kI);
        turnPID.setD(Drive.kD);
        // m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        // m_turningPIDController.setOutputRange(-1, 1);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(40);
        turnMotor.setSmartCurrentLimit(20);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        driveMotor.burnFlash();
        turnMotor.burnFlash();

        targetState.angle = new Rotation2d(turnEncoder.getPosition());
        driveEncoder.setPosition(0);
    }

    @Override
    public String getID() {
        return id;
    }

    @Override
    public Translation2d getOffset() {
        return offset;
    }

    private Rotation2d getAngle() {
        return new Rotation2d(turnEncoder.getPosition());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle().minus(angleOffset));
    }
  
    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), getAngle().minus(angleOffset));
    }
  
    @Override
    public void setTargetState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(angleOffset);
    
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            getAngle());
    
        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turnPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    
        targetState = desiredState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber(id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(id + "/Target Velocity", targetState.speedMetersPerSecond);
    }
}
