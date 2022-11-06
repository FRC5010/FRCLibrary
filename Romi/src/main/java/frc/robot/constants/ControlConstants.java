/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class ControlConstants {
    static enum ButtonNums {
        NO_BUTTON, A_BUTTON, B_BUTTON, X_BUTTON, Y_BUTTON, LEFT_BUMPER, RIGHT_BUMPER, START_BUTTON,
        BACK_BUTTON,LEFT_STICK_BUTT, RIGHT_STICK_BUTT;
    }
    static enum AxisNums {
    LEFT_X, LEFT_Y, 
     L_TRIGGER, 
     R_TRIGGER, 
    RIGHT_X, RIGHT_Y
    }
    static enum POVDirs {
        UP, RIGHT, DOWN, LEFT 
    }
    static enum Motors{
        NO_MOTOR, M1, M2, M3, M4, M5, M6, M7, M8, M9, M10, M11, M12, M13, M14, M15, M16, M17, M18;
    }
    static enum Pneumatics{
        S0, S1, S2, S3, S4, S5, S6;
    }
    static enum DioPorts {
        Port0, Port1, Port2, Port3, Port4, Port5, Port6, Port7, Port8, Port9;
    }
    //DIO Ports
    public static int limitSwitch = DioPorts.Port0.ordinal();
    public static int BB1 = DioPorts.Port0.ordinal();
    public static int BB2 = DioPorts.Port1.ordinal();
    public static I2C.Port i2cPort = I2C.Port.kOnboard;
    //public static int BB2 = DioPorts.Port2.ordinal();

    /*
    Motors
        M1: Left Drive
        M2: Right Drive
        M3: Left Drive
        M4: Right Drive
        M5: Right Winch
        M6: Left Winch
        M7: Intake Motor Upper
        M8: Diagonal Upper Indexer
        M9: Diagonal Lower Indexer
        M10: Vertical Long Indexer
        M11: Vertical Short Indexer
        M12: Hood
        M13: Feeder Left Flywheel
        M14: Left Flywheel
        M15: Right Flywheel
        M16: Turret
    */

    // Motor
    public static int leftDrive1M = Motors.M1.ordinal();
    public static int rightDrive1M = Motors.M2.ordinal();
    public static int leftDrive3M = Motors.M3.ordinal();
    public static int rightDrive3M = Motors.M4.ordinal();
    public static int rightWinchM = Motors.M5.ordinal();
    public static int leftWinchM = Motors.M6.ordinal();
    public static int intakeM = Motors.M7.ordinal();
    public static int diagonalUpperM = Motors.M8.ordinal();
    public static int diagonalLowerM = Motors.M9.ordinal();
    public static int verticalLongM = Motors.M10.ordinal();
    public static int verticalShortM = Motors.M11.ordinal();
    public static int hoodM = Motors.M12.ordinal();
    public static int feederWheelM = Motors.M13.ordinal();
    public static int leftFlyWheelM = Motors.M14.ordinal();
    public static int rightFlyWheelM = Motors.M15.ordinal();
    public static int turretM = Motors.M16.ordinal();

    /*
        Intake Double Solonoid
            Slots 0 and 1

        Climb Double Solonoid
            Slots 2 and 3
    
    */


    // Pneumatics
    public static int slot0P = Pneumatics.S0.ordinal();
    public static int slot1P = Pneumatics.S1.ordinal();
    public static int slot2P = Pneumatics.S2.ordinal();
    public static int slot3P = Pneumatics.S3.ordinal();

    // Driver
    public static int driverJoystick = 0;
    public static int throttle = AxisNums.LEFT_Y.ordinal();
    public static int steer = AxisNums.RIGHT_X.ordinal();
    public static int steerY = AxisNums.RIGHT_Y.ordinal();
    public static int outtakeAxis = AxisNums.L_TRIGGER.ordinal();
    public static int intakeAxis = AxisNums.R_TRIGGER.ordinal();

    //public static int rightClimbArmDown = ButtonNums.A_BUTTON.ordinal();
    public static int toggleLL = ButtonNums.A_BUTTON.ordinal();
    public static int takeSnapshot = ButtonNums.B_BUTTON.ordinal();
    //public static int rightClimbArmUp = ButtonNums.B_BUTTON.ordinal();
    //public static int leftClimbArmDown = ButtonNums.X_BUTTON.ordinal();
    //public static int leftClimbArmUp = ButtonNums.Y_BUTTON.ordinal();
    public static int upperFender = ButtonNums.LEFT_BUMPER.ordinal(); // unused
    public static int driveYEET = ButtonNums.LEFT_BUMPER.ordinal();
    
    //public static int toggleIntake = ButtonNums.RIGHT_BUMPER.ordinal();
    //automatically raises and lowers intake whenever we intake or outtake

    public static int calibrate = ButtonNums.START_BUTTON.ordinal();
    public static int climbTime = ButtonNums.BACK_BUTTON.ordinal();

    
    //public static int toggleDrive = ButtonNums.LEFT_STICK_BUTT.ordinal();
    //public static int toggleLed = ButtonNums.RIGHT_STICK_BUTT.ordinal();
    //public static int toggleLL = ButtonNums.RIGHT_STICK_BUTT.ordinal();
    
    

    public static int incThrottleFactor = POVDirs.UP.ordinal() * 90;
    public static int decThrottleFactor = POVDirs.DOWN.ordinal() * 90;
    public static int decSteerFactor = POVDirs.LEFT.ordinal() * 90;
    public static int incSteerFactor = POVDirs.RIGHT.ordinal() * 90;

    //Operator
    public static int operatorJoystick = 1;
    public static int leftClimbArm = AxisNums.LEFT_Y.ordinal();
    public static int joystickIndexer = AxisNums.LEFT_Y.ordinal(); 
    public static int rightClimbArm = AxisNums.RIGHT_Y.ordinal();
    public static int spinHood = AxisNums.RIGHT_Y.ordinal();
    public static int turnTurret = AxisNums.RIGHT_X.ordinal();
    public static int toggleClimb = AxisNums.L_TRIGGER.ordinal(); 
    public static int lockAndLoadButton = AxisNums.R_TRIGGER.ordinal();

    public static int fender2Button = ButtonNums.A_BUTTON.ordinal(); // 3/25/2022 added arms down for comp at tippie
    public static int hoodDown = ButtonNums.B_BUTTON.ordinal(); 
    public static int toggleReject = ButtonNums.X_BUTTON.ordinal();
    public static int hoodUp = ButtonNums.Y_BUTTON.ordinal(); 
    public static int fenderButton = ButtonNums.LEFT_BUMPER.ordinal();
    public static int launchButton = ButtonNums.RIGHT_BUMPER.ordinal();
    //public static int fenderButton2 = ButtonNums.RIGHT_STICK_BUTT.ordinal();
    //public static int lockAndLoadButton = ButtonNums.LEFT_STICK_BUTT.ordinal();

    //public static int overrideIntake = ButtonNums.START_BUTTON.ordinal();

    public static int defaultShoot = ButtonNums.START_BUTTON.ordinal();
    
    

    public static int incShooter = POVDirs.UP.ordinal() * 90;
    public static int decShooter = POVDirs.DOWN.ordinal() * 90;
    public static int shotAdjustmentUp = POVDirs.RIGHT.ordinal() * 90;
    public static int shotAdjustmentDown = POVDirs.LEFT.ordinal() * 90;

    //test button for disable ball reject
    

    // Shuffleboard constants
    public static String SBTabDriverDisplay = "Driver Display";
    public static String SBTabVisionDisplay = "Vision Display";
    public static String SBTabClimbDisplay = "Climb Display";
    public static String SBTabDiagnostics = "Diagnostics";
    public static int shooterColumn = 0;
    public static int hoodColumn = 2;

    public static int driverColumn = 4;
    public static int autoColumn = 4;
    public static int limelightColumn = 0;
    public static int intakeVisionColumn = 0;
    public static int shooterVisionColumn = 6;

    public static int autoNavButton = ButtonNums.X_BUTTON.ordinal();
    public static int driveTrainCurrentLimit = 38;
    public static int babyNeoCurrentLimit = 20;
    public static int neoCurrentLimit = 38;
    public static Color allianceColor = Color.kBlack;
    public static Color opposingColor = Color.kBlack;
    
    
    public static boolean setupSingleDriver(Joystick operator){
        if(operator.getType() == HIDType.kUnknown){
            throttle = AxisNums.LEFT_Y.ordinal();
            steer = AxisNums.RIGHT_X.ordinal();
        
            //startClimb = ButtonNums.LEFT_BUMPER.ordinal();
        
            launchButton = ButtonNums.A_BUTTON.ordinal();
            autoNavButton = ButtonNums.X_BUTTON.ordinal();
            return true;
        }
        return false;
    }
}
