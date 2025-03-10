package com.stuypulse.robot.subsystems.plant;

import static com.stuypulse.robot.constants.Ports.Plant.*;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
* @author Samuel Chen
* @author Carmin Vuong
* @author Jiayu Yan
* @author Tracey Lin
*
*/
public class PlantImpl extends Plant {
    private final DoubleSolenoid solenoid;

    public PlantImpl() {
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD, REVERSE);
        disengage();
    }

    public void engage() {
        solenoid.set(Value.kReverse);
    }

    public void disengage() {
        solenoid.set(Value.kForward);
    }

    @Override
    public void periodic() {
        Settings.putBoolean("Is Engaged", solenoid.get()==Value.kReverse);
    }
}
