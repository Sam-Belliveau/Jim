package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
public class ArmIntake extends ArmRoutine {
    
    public ArmIntake() {
        super(Routine.INTAKE, Manager.getInstance()::getIntakeTrajectory);
    }
}
