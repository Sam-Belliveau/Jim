package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase{
    private final SwerveDrive swerve;
    private final Supplier<Pose2d> targetPoses;

    // Holonomic control
    private final Controller xController;
    private final Controller yController;
    private final AngleController angleController;

    private final BStream aligned;
    
    public SwerveDriveToPose(Supplier<Pose2d> targetPoses){
        this.swerve = SwerveDrive.getInstance();
        this.targetPoses = targetPoses;

        xController = new PIDController(Translation.P,Translation.I,Translation.D);
        yController = new PIDController(Translation.P, Translation.I, Translation.D);
        angleController = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D);
        
        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));
        
        addRequirements(swerve);
    }

    private boolean isAligned() {
        return xController.isDone(Alignment.ALIGNED_THRESHOLD_X.get())
            && yController.isDone(Alignment.ALIGNED_THRESHOLD_Y.get())
            && angleController.isDoneDegrees(Alignment.ALIGNED_THRESHOLD_ANGLE.get());
    }

    @Override
    public void execute() {

        Pose2d currentState = Odometry.getInstance().getPose();
        Pose2d targetPose = targetPoses.get();

        boolean alignY = xController.isDone(Units.inchesToMeters(6));

        xController.update(targetPose.getX(), currentState.getX());
        yController.update(targetPose.getY(), currentState.getY());
        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            alignY ? 0 : xController.getOutput(),
            alignY ? yController.getOutput() : 0,
            angleController.update(Angle.fromRotation2d(targetPose.getRotation()), Angle.fromRotation2d(currentState.getRotation())),
            currentState.getRotation()
        );

        swerve.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public boolean isFinished(){
        return aligned.get();
    }

    public void end(boolean interupted) {
        swerve.stop();
    }
    
}
