package com.lcrobotics.easyftclib.CommandCenter.driveTrain;

import android.support.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *
 */
public class Drive extends CommandImpl {
    public static int x = 5;
    public double distance;
    public double angle;
    Orientation currentHeading;
    /**
     *
     * @param distance
     * @param angle
     */
    public Drive(double distance, double angle) {
        this.distance = distance;
        this.angle = angle;
    }

    @Override
    public int update() {
        if (gyroscope == null) {

        } else {
            double zAngle = gyroscope.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle;

        }
        return 0;
    }

    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        return super.initialize(parameters);
    }
}
