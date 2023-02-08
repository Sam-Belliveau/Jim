package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.filters.Derivative;

public class Feedthrough extends Controller {

    private Derivative derivative;

    @Override
    protected double calculate(double setpoint, double measurement) {
        return derivative.get(setpoint);
    }
    
}
