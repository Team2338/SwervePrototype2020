package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.lib.control.DifferentialModule;
import team.gif.lib.math.SwerveMath;
import team.gif.lib.math.Vector2d;
import team.gif.robot.Constants;
import team.gif.robot.OI;
import team.gif.robot.RobotMap;
import team.gif.robot.commands.drivetrain.Drive;

// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
// https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance = null;

    private DifferentialModule FRModule, FLModule, RLModule, RRModule;

    //private PigeonIMU pigeon;

    //private double[] ypr_deg;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    private Drivetrain() {
        super();

        FRModule = new DifferentialModule(RobotMap.FR_DRIVE_ID_1, RobotMap.FR_DRIVE_ID_2, RobotMap.FR_ENCODER_ID, new Vector2d(11.375, 11.375));
        //FLModule = new DifferentialModule(RobotMap.FL_DRIVE_ID_1, RobotMap.FL_DRIVE_ID_2, RobotMap.FL_ENCODER_ID, new Vector2d(-8.875, 8.875));
        //RLModule = new DifferentialModule(RobotMap.RL_DRIVE_ID_1, RobotMap.RL_DRIVE_ID_2, RobotMap.RL_ENCODER_ID, new Vector2d(-11.375, -11.375));
        //RRModule = new DifferentialModule(RobotMap.RR_DRIVE_ID_1, RobotMap.RR_DRIVE_ID_2, RobotMap.RR_ENCODER_ID, new Vector2d(8.875, -8.875));

        //pigeon = new PigeonIMU(0);

        //ypr_deg = new double[3];
    }

    public void set(double robotHeading, Vector2d transVecF, double rotVal) {
//        Vector2d[] driveVecs = SwerveMath.calculateDriveVector(robotHeading, transVecF, rotVal,
//                FRModule.getModulePos(), FLModule.getModulePos(), RLModule.getModulePos(), RRModule.getModulePos());
        Vector2d[] driveVecs = SwerveMath.calculateDriveVector(robotHeading, transVecF, rotVal,
                FRModule.getModulePos(), RLModule.getModulePos());
        FRModule.setVectorPercent(driveVecs[0]);
        //FLModule.setVectorPercent(driveVecs[1]);
        //RLModule.setVectorPercent(driveVecs[1]);
        //RRModule.setVectorPercent(driveVecs[3]);
    }

    /*public double[] getYawPitchRollDeg() {
        pigeon.getYawPitchRoll(ypr_deg);
        return ypr_deg;
    }
    public double getHeading() {
        return Math.toRadians(getYawPitchRollDeg()[0]);
    }*/

    public double[] getModuleAngles() {
        return new double[]{FRModule.getAngle(), RLModule.getAngle()};
    }


}
