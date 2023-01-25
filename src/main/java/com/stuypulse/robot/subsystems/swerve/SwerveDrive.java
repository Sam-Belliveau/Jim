package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.subsystems.ISwerveModule;
import com.stuypulse.robot.subsystems.Odometry;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports.Swerve.FrontRight;
import com.stuypulse.robot.constants.Ports.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Ports.Swerve.BackRight;
import com.stuypulse.robot.constants.Ports.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private static SwerveDrive instance = null;
    
    public static SwerveDrive getInstance() {
        if (instance == null) {
            if (RobotBase.isReal()) {
                instance = new SwerveDrive(
                    new MAX_SwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE),
                    new MAX_SwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER, FrontLeft.ABSOLUTE_OFFSET, Swerve.FrontLeft.DRIVE),
                    new MAX_SwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE),
                    new MAX_SwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE)
                );
            } else {
                
            }
        }

        return instance;
    }

    /** MODULES */
    private final ISwerveModule[] modules;

    /** SENSORS */
    private final AHRS gyro;
    
    private final SwerveDriveKinematics kinematics;
    
    private final FieldObject2d[] module2ds;

    public SwerveDrive(ISwerveModule... modules) {
        this.modules = modules;

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(getModuleLocations());
        
        module2ds = new FieldObject2d[modules.length];       
        for(int i = 0; i < module2ds.length; ++i){
            module2ds[i] = Odometry.getInstance().getField().getObject(modules[i].getID()+"-2d");
        }
        
    }
    
    private Translation2d[] getModuleLocations() {
        Translation2d[] locations = new Translation2d[modules.length];    
        
        for(int i = 0; i < modules.length; ++i) {
            locations[i] = modules[i].getLocation();
        }
        
        return locations;
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private ISwerveModule getModule(String id) {
        for (ISwerveModule module : modules) 
            if (module.getID().equals(id)) {
                return module;
        }
        throw new IllegalArgumentException("Couldn't find the module with id \"" + id + "\"");
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }
    
    /** MODULE STATES API **/
    public void drive(Vector2D velocity, double omega) {
        // ADD 254 DRIFT FIX!
        
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.y, -velocity.x,
                -omega,
                Odometry.getInstance().getAngle());

        setChassisSpeeds(speeds, false);
    }
    
    public void setChassisSpeeds(ChassisSpeeds robotSpeed, boolean fieldRelative) {
        if (fieldRelative) {
            robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                robotSpeed,
                Odometry.getInstance().getAngle());
        }

        setModuleStates(kinematics.toSwerveModuleStates(robotSpeed));
    }
    
    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_SPEED);
        
        for(int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds(), true);
    }
    
    /** GYRO API **/
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }
    
    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }
    
    /* Kinematics*/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    
    @Override
    public void periodic() {
        Odometry.getInstance().updatePose(
            getGyroAngle(),
            getModulePositions()
        );

        Pose2d pose = Odometry.getInstance().getPose();
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getLocation().rotateBy(Odometry.getInstance().getAngle())),
                modules[i].getState().angle.plus(Odometry.getInstance().getAngle())
            ));
        }

        SmartDashboard.putNumber("Swerve/Gyro Angle", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Pitch", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Roll", getGyroRoll().getDegrees());
    }
    
    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.DT));
    }
    
}