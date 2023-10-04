package com.pocolifo.robobase.control.input.buttonhandles;

import com.qualcomm.robotcore.hardware.Gamepad;

public class PsButtonHandle extends ButtonHandle {
    private Gamepad g;
    public PsButtonHandle(Gamepad g) {
        this.g = g;
    }
    public float get() {
        return g.ps?1f:0f;
    }
}