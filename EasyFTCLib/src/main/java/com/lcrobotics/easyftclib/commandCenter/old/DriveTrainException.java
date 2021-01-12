package com.lcrobotics.easyftclib.commandCenter.old;

/**
 * DriveTrain exception, thrown if there's an error
 */
public class DriveTrainException extends Exception {
    public DriveTrainException(String errorMessage){
        super(errorMessage);
    }
}
