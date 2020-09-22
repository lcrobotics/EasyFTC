package com.lcrobotics.easyftclib.Unused;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public abstract class CustomOpMode extends LinearOpMode {
    private void addAnnotatedOpMode(Class<OpMode> clazz)
    {
        if (clazz.isAnnotationPresent(TeleOp.class))
        {
        }
    }
}
