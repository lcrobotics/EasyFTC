package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

public class BoolAndMatrix {
    public boolean targetVisible;
    public OpenGLMatrix lastLocation;
    public BoolAndMatrix(boolean targetVisible, OpenGLMatrix lastLocation){
        this.targetVisible = targetVisible;
        this.lastLocation = lastLocation;
    }
}
