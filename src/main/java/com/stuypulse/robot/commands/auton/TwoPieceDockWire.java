package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceWithPlant;
import com.stuypulse.robot.subsystems.Manager.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceDockWire extends SequentialCommandGroup {

    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double ALIGNMENT_TIME = 1.0;
    private static final double ENGAGE_TIME = 3.0;

    private static final PathConstraints INTAKE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints SCORE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints DOCK_CONSTRAINTS = new PathConstraints(2, 2);

    public TwoPieceDockWire(){
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece + Dock Wire", INTAKE_PIECE_CONSTRAINTS, SCORE_PIECE_CONSTRAINTS, DOCK_CONSTRAINTS),
            "Intake Piece" , "Score Piece", "Dock"
        );

        // initial setup
        addCommands(
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_IN),
            new ManagerSetIntakeSide(IntakeSide.FRONT),
            new ManagerSetScoreSide(ScoreSide.OPPOSITE)
        );

        // score first piece
        addCommands(
            new ArmReady(),
            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        // drive to second game piece and intake
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetNodeLevel(NodeLevel.MID),

            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece"))
                    .robotRelative()
                    .addEvent("ReadyIntakeOne",new ArmIntake().andThen(new IntakeAcquire()))
                    .withEvents(),

            new IntakeWaitForPiece().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );
        
        // drive to grid and score game piece
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece"))
                    .fieldRelative()
                    .addEvent("ReadyArmOne", new ArmReady())
                    .withEvents(),

            new ManagerSetScoreIndex(7),
            new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),
            new ArmScore(),
            new IntakeDeacquire(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        // dock and engage
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Dock"))
                    .fieldRelative()
                    .addEvent("ArmNeutral", new ArmNeutral())
                    .withEvents(),

            new SwerveDriveBalanceWithPlant().withTimeout(ENGAGE_TIME),
            new PlantEngage()
        );
    }
}
