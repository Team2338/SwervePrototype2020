package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.OI;
import team.gif.robot.RobotMap;
import team.gif.robot.commands.drivetrain.Drive;

// https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
// https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance = null;

    private static final TalonSRX angleDriveMotor = new TalonSRX(RobotMap.DRIVE_ANGLE);
    private static final TalonSRX speedDriveMotor = new TalonSRX(RobotMap.DRIVE_SPEED);

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    private Drivetrain() {
        super();

        //leftMaster.setInverted(true);
        //leftSlave.setInverted(true);
//        rightMaster.setInverted(Constants.IS_INVERTED_DRIVE_RIGHT_MASTER);
//        rightSlave.setInverted(Constants.IS_INVERTED_DRIVE_RIGHT_SLAVE);

        angleDriveMotor.setNeutralMode(NeutralMode.Brake);
        speedDriveMotor.setNeutralMode(NeutralMode.Brake);

        //leftSlave.follow(leftMaster);
        //rightSlave.follow(rightMaster);
    }

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    //ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);
    ChassisSpeeds speeds;
    public void drive(double x1, double y1, double x2) {
        speeds = new ChassisSpeeds(x1, y1, x2);
    }

    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

    /*
    // The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));

    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
*/

    // Example module states
    SwerveModuleState frontLeftState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19));
    SwerveModuleState frontRightState = new SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81));
    SwerveModuleState backLeftState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44));
    SwerveModuleState backRightState = new SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56));

    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(
            frontLeftState, frontRightState, backLeftState, backRightState);

    // Getting individual speeds
    double forward = chassisSpeeds.vxMetersPerSecond;
    double sideways = chassisSpeeds.vyMetersPerSecond;
    double angular = chassisSpeeds.omegaRadiansPerSecond;


    // OLD BELOW
    /*public void setSpeed(double left, double right) {

        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }*/

    //@Override
    //protected void initDefaultCommand() {
    //    setDefaultCommand(new Drive(Drivetrain.getInstance()));
    //}
}
