package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Motors.Intake.BACK_MOTOR;
import static com.stuypulse.robot.constants.Motors.Intake.FRONT_MOTOR;
import static com.stuypulse.robot.constants.Settings.Intake.CONE_BACK_ROLLER;
import static com.stuypulse.robot.constants.Settings.Intake.CONE_FRONT_ROLLER;
import static com.stuypulse.robot.constants.Settings.Intake.CUBE_BACK_ROLLER;
import static com.stuypulse.robot.constants.Settings.Intake.CUBE_FRONT_ROLLER;
import static com.stuypulse.robot.constants.Settings.Intake.STALL_CURRENT;
import static com.stuypulse.robot.constants.Settings.Intake.STALL_TIME;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.IArm;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends IIntake{

    private CANSparkMax frontMotor; 
    private CANSparkMax backMotor;

    private DigitalInput frontLeftSensor;
    private DigitalInput frontRightSensor;
    private DigitalInput backLeftSensor;
    private DigitalInput backRightSensor;

    private BStream stalling;

    public Intake(){
       
        frontMotor = new CANSparkMax(Ports.Intake.FRONT_MOTOR, MotorType.kBrushless);
        backMotor = new CANSparkMax(Ports.Intake.BACK_MOTOR, MotorType.kBrushless);

        FRONT_MOTOR.configure(frontMotor);
        BACK_MOTOR.configure(backMotor);

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounce.Rising(STALL_TIME))
            .polling(Settings.DT);

        frontLeftSensor = new DigitalInput(Ports.Intake.FRONT_LEFT_SENSOR);
        frontRightSensor = new DigitalInput(Ports.Intake.FRONT_RIGHT_SENSOR);
        backLeftSensor = new DigitalInput(Ports.Intake.BACK_LEFT_SENSOR);
        backRightSensor = new DigitalInput(Ports.Intake.BACK_RIGHT_SENSOR);
    }

    // CONE DETECTION (stall detection)

    private boolean isMomentarilyStalling() {
        return Math.abs(frontMotor.getOutputCurrent()) > STALL_CURRENT.doubleValue();
    }

    private boolean isStalling() {
        return stalling.get();
    }

    // CUBE DETECTION (ir sensors)

    private boolean hasCubeFront() {
        return !frontLeftSensor.get()||!frontRightSensor.get();
    }
    private boolean hasCubeBack() {
        return !backLeftSensor.get()||!backRightSensor.get();
    }
    private boolean hasCube() {
        return isFlipped()? hasCubeBack() : hasCubeFront();
    }

    // WRIST ORIENTATION

    private boolean isFlipped() {
        IArm arm = IArm.getInstance();
        return arm.getWristAngle().toRadians() > Math.PI /2 || arm.getWristAngle().toRadians() < 3 * Math.PI / 2;
    }

    // INTAKING MODES

    public void cubeIntake(){
        if (isFlipped()) {
            frontMotor.set(-CUBE_FRONT_ROLLER.get());
            backMotor.set(-CUBE_BACK_ROLLER.get());
        } else {
            frontMotor.set(CUBE_FRONT_ROLLER.get());
            backMotor.set(CUBE_BACK_ROLLER.get());
        }
    }

    public void coneIntake() {
        if (isFlipped()) {
            frontMotor.set(-CONE_FRONT_ROLLER.get());
            backMotor.set(CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(CONE_FRONT_ROLLER.get());
            backMotor.set(-CONE_BACK_ROLLER.get());
        }
    }

    public void cubeOuttake(){
        if (isFlipped()) {
            frontMotor.set(CUBE_FRONT_ROLLER.get());
            backMotor.set(CUBE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-CUBE_FRONT_ROLLER.get());
            backMotor.set(-CUBE_BACK_ROLLER.get());
        }
    }

    public void coneOuttake(){
        if (isFlipped()) {
            frontMotor.set(CONE_FRONT_ROLLER.get());
            backMotor.set(-CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-CONE_FRONT_ROLLER.get());
            backMotor.set(CONE_BACK_ROLLER.get());
        }
    }

    @Override
    public void periodic(){
        if (isStalling() || hasCube()) {
            frontMotor.stopMotor();
            backMotor.stopMotor();
        }
        SmartDashboard.putBoolean("Intake/Is Flipped", isFlipped());
        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/Has Cube", hasCube());
    }

}