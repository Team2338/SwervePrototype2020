/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team.gif.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double MODULE_ANGLE_P = 0.1;
    public static final double MODULE_ANGLE_I = 0.0;
    public static final double MODULE_ANGLE_D = 0.0;

    public static final double MAX_DRIVE_VELOCITY = 100.0; //in/s

    public static final double WHEEL_DIAMETER = 4.0;

    public static final double DRIVE_RATIO = 7.5;

    public static final double FR_kV = 473; // rpm/v
}
