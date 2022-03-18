// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * !! Make sure hood is correctly positioned before powering on/off !!
 */

/* TODO: 
autonomous yikes
    start with 1 ball, drive to collect 2
    after shooting 2, drive to collect 2
    shoot 2 and profit
        4 total balls
        implement limelight aiming during autonomous
automate climber for real
    no time requirement for next stage of climbing, use encoders
    find encoder values for angles + max extension
clean up code
add hood to limelight code, set up encoder values
*/

package frc.robot;

import java.util.List;

import javax.tools.ForwardingFileObject;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

    // Constants
    private static final double EXTPULLUP = 0;
    private static final double ARTROTATETOGRAB = 0;

    private static final double DEADZONE = 0.1;
    private static final double SPEED_MOD = 1;
    private static final int TRIGGER = 1;
    private static final int THUMBBUTTON = 2;

    private static final double COLLECTORSPEED = 0.5;
    private static final double EXTCLIMBERSPEED = 0.75;
    private static final double ARTCLIMBERSPEED = 0.25;

    public static final double CLOSEDISTANCE = 114;
    public static final double MIDDISTANCE = 174;
    public static final double FARDISTANCE = 228;

    public static final double CLOSE_ANGLE = 0.0;
    public static final double MID_ANGLE = 5.158;
    public static final double FAR_ANGLE = 16.453;
    public static final double FLAP_DEADZONE = 0.1;

    // the autonomous garbage
    // public static final double CHASSISWHEELWIDTH = Units.inchesToMeters(20); //
    // inches but now meters
    public static final double KSVOLTS = 0.22;
    public static final double KVVOLTSECONDSPERMETER = 1.98;
    public static final double KAVOLTSECONDSSQUAREDPERMETER = 0.2;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower, meters & seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kPDriveVel = 8.5;
    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // PID Coefficients
    // Don't touch
    public static final double PID_P = .006; // was 0.01, better at .009
    public static final double PID_I = 0;
    public static final double PID_D = 0;
    public static final double PID_IZ = 0;
    public static final double PID_FF = 0.00011; // was 0.0, better at 0.00001
    public static final double PID_MAXOUTPUT = 1;
    public static final double PID_MINOUTPUT = -1;

    // Joysticks
    private final Joystick joyE = new Joystick(1);
    private final Joystick joyL = new Joystick(0);
    private final Joystick joyR = new Joystick(2);

    // Motor Controllers
    private final WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(9);
    private final WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(5);
    private final WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(2);
    private final WPI_VictorSPX backRightMotor = new WPI_VictorSPX(3);
    private final WPI_VictorSPX topCollectorMotor = new WPI_VictorSPX(6);
    private final WPI_VictorSPX bottomCollectorMotor = new WPI_VictorSPX(10);
    private final WPI_VictorSPX topArticulatingClimber = new WPI_VictorSPX(7);
    private final WPI_VictorSPX bottomArticulatingClimber = new WPI_VictorSPX(11);
    private final WPI_VictorSPX topExtendingClimber = new WPI_VictorSPX(4); // EXT
    private final WPI_VictorSPX bottomExtendingClimber = new WPI_VictorSPX(12); // EXT2
    private final WPI_VictorSPX hoodMotor = new WPI_VictorSPX(8);
    private final CANSparkMax shooterMotor = new CANSparkMax(1, MotorType.kBrushless);

    private final SparkMaxPIDController shooterPID = shooterMotor.getPIDController();

    private final Counter flapEncoder = new Counter(new DigitalInput(9));
    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private final Encoder articulatingEncoder = new Encoder(5, 6);
    private final Encoder extendingEncoder = new Encoder(7, 8);
    private final Encoder leftEncoder = new Encoder(1, 2, true); // inverted
    private final Encoder rightEncoder = new Encoder(3, 4, false); // not inverted

    // Gyroscope
    AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Motor Controller Groups
    private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);
    private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    // private final DriveSubsystem robotDriveSubsystem = new DriveSubsystem();
    private final MotorControllerGroup extendingClimbers = new MotorControllerGroup(topExtendingClimber,
            bottomExtendingClimber);
    private final MotorControllerGroup articulatingClimbers = new MotorControllerGroup(topArticulatingClimber,
            bottomArticulatingClimber);

    // private final DifferentialDriveOdometry odometry = new
    // DifferentialDriveOdometry(gyro.getRotation2d());
    // private final DifferentialDriveKinematics kinematics = new
    // DifferentialDriveKinematics(CHASSISWHEELWIDTH);

    // Limelight
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double limelightKP = -0.1;
    double limelightMinCommand = 0.1;
    double limelightLeftSteer = 0.0;
    double limelightRightSteer = 0.0;

    // Flap Values
    double desiredFlapAngle = MID_ANGLE;
    double currentFlapAngle = 0.0;

    @Override
    public void robotInit() {

        configLimelight();
        leftMotorGroup.setInverted(true);
        rightMotorGroup.setInverted(false);

        // Set PID Coefficients
        shooterPID.setP(PID_P);
        shooterPID.setI(PID_I);
        shooterPID.setD(PID_D);
        shooterPID.setIZone(PID_IZ);
        shooterPID.setFF(PID_FF);
        shooterPID.setOutputRange(PID_MINOUTPUT, PID_MAXOUTPUT);

        // Print PID Coefficients to SmartDashboard
        SmartDashboard.putNumber("P Gain", PID_P);
        SmartDashboard.putNumber("Feed Forward", PID_FF);

        resetEncoders();
    }

    // public Command getAutonomousCommand() {

    // // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    // new SimpleMotorFeedforward(
    // Constants.KSVOLTS,
    // Constants.KVVOLTSECONDSPERMETER,
    // Constants.KAVOLTSECONDSSQUAREDPERMETER),
    // kinematics,
    // 10);

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    // Constants.kMaxSpeedMetersPerSecond,
    // Constants.kMaxAccelerationMetersPerSecondSquared)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(kinematics)
    // // Apply the voltage constraint
    // .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // // Pass config
    // config);

    // RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory,
    // robotDriveSubsystem::getPose,
    // new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    // new SimpleMotorFeedforward(Constants.KSVOLTS,
    // Constants.KVVOLTSECONDSPERMETER,
    // Constants.KAVOLTSECONDSSQUAREDPERMETER),
    // kinematics,
    // robotDriveSubsystem::getWheelSpeeds,
    // new PIDController(Constants.kPDriveVel, 0, 0), new
    // PIDController(Constants.kPDriveVel, 0, 0),
    // robotDriveSubsystem::tankDriveVolts, robotDriveSubsystem);

    // // Reset odometry to the starting pose of the trajectory.
    // robotDriveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> robotDrive.tankDrive(0, 0));
    // }

    // double getHeading = gyro.getRotation2d().getDegrees();
    // double getTurnRate = -gyro.getRate();

    @Override
    public void teleopInit() {

        // How long the robot has been turned on when teleop is first enabled
        // TODO: remove me
        double teleopStartTime = Timer.getFPGATimestamp();

    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Flap Distance?", flapEncoder.getDistance());

        SmartDashboard.putNumber("Flap Encoder", (currentFlapAngle / 360));
        SmartDashboard.putNumber("Articulating Encoder", articulatingEncoder.getDistance());
        SmartDashboard.putNumber("Extending Encoder", extendingEncoder.getDistance());

        // Set PID Coefficients
        shooterPID.setP(SmartDashboard.getNumber("P Gain", PID_P));
        shooterPID.setI(PID_I);
        shooterPID.setD(PID_D);
        shooterPID.setIZone(PID_IZ);
        shooterPID.setFF(SmartDashboard.getNumber("Feed Forward", PID_FF));
        shooterPID.setOutputRange(PID_MINOUTPUT, PID_MAXOUTPUT);

        // Buttons
        boolean spinShooterButton = joyE.getTop();
        boolean halfsiesL = joyL.getRawButton(TRIGGER);
        boolean halfsiesR = joyR.getRawButton(TRIGGER);
        boolean initialCollectionLButton = joyL.getRawButton(THUMBBUTTON);
        boolean initialCollectionRButton = joyR.getRawButton(THUMBBUTTON);
        boolean feederCollectionButton = joyE.getRawButton(THUMBBUTTON);
        boolean hoodAdjustForward = joyE.getRawButton(12);
        boolean hoodAdjustBackward = joyE.getRawButton(10);
        boolean ejectButtonL = joyL.getRawButton(3);
        boolean ejectButtonR = joyR.getRawButton(3);
        boolean resetHoodDegreesTest = joyL.getRawButton(6);

        // Limelight Buttons
        boolean limelightAlignButtonL = joyL.getRawButton(4);
        boolean limelightAlignButtonR = joyR.getRawButton(4);
        boolean limelightShootButton = joyE.getRawButton(TRIGGER);

        // Manual Climber Buttons
        boolean articulatingClimberButton = joyE.getRawButton(5);
        boolean articulatingClimberOtherwayButton = joyE.getRawButton(3);
        boolean windClimberButton = joyE.getRawButton(6);
        boolean unwindClimberButton = joyE.getRawButton(4);

        // Automated Climber Buttons: obsolete
        boolean firstButton = joyE.getRawButton(11);
        boolean secondButton = joyE.getRawButton(8);
        boolean thirdButton = joyE.getRawButton(7);

        // Driving Variables
        double leftHalfSpeed = 0;
        double rightHalfSpeed = 0;

        // Updating Variables
        double shooterSpeedManual = (joyE.getRawAxis(3) / 4) + 0.75; // converts [-1, 1] to [-1/4, 1/4] to [0.5, 1]

        // Limelight Variables
        double limelightTV = table.getEntry("tv").getDouble(0);
        double limelightTX = table.getEntry("tx").getDouble(0);
        double limelightTY = table.getEntry("ty").getDouble(0);

        double limelightDistance = calculateLimelightDistance(limelightTY);
        double desiredShooterRPM = calculateLimelightRPM(limelightDistance);
        boolean limelightHasTarget = limelightTV > 0;

        SmartDashboard.putBoolean("Valid Target?", limelightHasTarget);

        /*
         * // Hold down reset encoder button for this to work
         * if (resetHoodDegreesTest) {
         * hoodResetMethod();
         * }
         */

        // Hood Adjustment Manual
        if (hoodAdjustForward) {
            hoodMotor.set(0.5);
            currentFlapAngle += flapEncoder.get();
        } else if (hoodAdjustBackward) {
            hoodMotor.set(-0.5);
            currentFlapAngle -= flapEncoder.get();
        } else {
            hoodMotor.set(0);
        }

        // Fancy Ternary Operators heck yeah
        leftHalfSpeed = halfsiesL ? 0.5 : 1;
        rightHalfSpeed = halfsiesR ? 0.5 : 1;

        // Drivetrain Controls: left and right joysticks
        if (Math.abs(joyL.getY()) > DEADZONE || Math.abs(joyR.getY()) > DEADZONE) {
            robotDrive.tankDrive(joyL.getY() * SPEED_MOD * leftHalfSpeed, joyR.getY() * SPEED_MOD * rightHalfSpeed);
        } else {
            robotDrive.tankDrive(0, 0);
        }

        // Limelight Alignment Controls
        if (limelightAlignButtonL || limelightAlignButtonR) {
            limelightLeftSteer = limelightSteeringAlign(limelightAlignButtonL, limelightAlignButtonR, limelightTX);
            limelightRightSteer = limelightSteeringAlign(limelightAlignButtonL, limelightAlignButtonR, limelightTX);

            robotDrive.tankDrive(limelightLeftSteer, limelightRightSteer);
        }
        adjustLimelightFlapAngle(limelightDistance);
        // Shooting Motor Controls
        if (limelightShootButton) {
            // Sets shooting motor PID to limelight's calculated RPM value when enabled
            shooterPID.setReference(desiredShooterRPM,
                    CANSparkMax.ControlType.kVelocity);
            adjustLimelightFlapAngle(limelightDistance);
            SmartDashboard.putNumber("Current Flap Angle", currentFlapAngle);
            SmartDashboard.putNumber("Desired Flap Angle", desiredFlapAngle);
        } else if (spinShooterButton) {
            // Else uses value from operator control
            shooterMotor.set(shooterSpeedManual);
        } else {
            shooterMotor.set(0);
        }

        boolean RPMinRange = (Math.abs(desiredShooterRPM - shooterEncoder.getVelocity()) < 75);

        // SmartDashboard Values for testing
        SmartDashboard.putBoolean("RPM IN RANGE", RPMinRange);
        SmartDashboard.putNumber("Limelight Distance", calculateLimelightDistance(limelightTY));
        SmartDashboard.putNumber("Limelight RPM Value", desiredShooterRPM);
        SmartDashboard.putNumber("Actual RPM", shooterEncoder.getVelocity());
        SmartDashboard.putBoolean("Limelight Shooting Enabled?", limelightShootButton);
        SmartDashboard.putNumber("Manual %", shooterSpeedManual);

        // Collection Controls
        if (initialCollectionLButton || initialCollectionRButton) {
            topCollectorMotor.set(-COLLECTORSPEED);
            bottomCollectorMotor.set(COLLECTORSPEED);
        } else if (feederCollectionButton) {
            topCollectorMotor.set(COLLECTORSPEED);
            bottomCollectorMotor.set(COLLECTORSPEED);
        } else if (ejectButtonL || ejectButtonR) {
            topCollectorMotor.set(-COLLECTORSPEED);
            bottomCollectorMotor.set(-COLLECTORSPEED);
        } else {
            topCollectorMotor.set(0);
            bottomCollectorMotor.set(0);
        }

    }

    public double limelightSteeringAlign(boolean limelightAlignButtonL, boolean limelightAlignButtonR,
            double limelightTX) {
        limelightLeftSteer = 0.0;
        limelightRightSteer = 0.0;

        double limelightHeadingError = -limelightTX;
        double limelightAlignmentAdjust = 0.0;

        if (limelightTX > 1.0) {
            limelightAlignmentAdjust = (limelightKP * limelightHeadingError) - limelightMinCommand;
        } else if (limelightTX < 1.0) {
            limelightAlignmentAdjust = (limelightKP * limelightHeadingError) + limelightMinCommand;
        }

        return limelightAlignmentAdjust;
    }

    public double calculateLimelightDistance(double limelightTY) {
        final double limelightMountingHeight = 24; // inches
        final double limelightMountingAngle = 32.5; // degrees rotated back from vertical apparently
        final double limelightTargetHeight = 104; // inches
        final double limelightToBumper = 11.25; // horizontal distance from limelight's camera to outside of bumper
        double limelightDistance = 0.0;

        double limelightTotalAngle = Math.toRadians(limelightMountingAngle + limelightTY);

        // Limelight distance from target in inches
        limelightDistance = ((limelightTargetHeight - limelightMountingHeight) / Math.tan(limelightTotalAngle))
                - limelightToBumper;
        return limelightDistance;
    }

    public double calculateLimelightRPM(double limelightDistance) {
        double limelightShootingRPM = 0;

        if (limelightDistance <= CLOSEDISTANCE) {
            limelightShootingRPM = (2870 * Math.pow(1.002, limelightDistance));
        } else if (limelightDistance <= MIDDISTANCE) {
            limelightShootingRPM = (2713 * Math.pow(1.002, limelightDistance));
        } else if (limelightDistance <= FARDISTANCE) {
            limelightShootingRPM = (3577 * Math.pow(1.001, limelightDistance));
        } else {
            limelightShootingRPM = 0;
        }

        return limelightShootingRPM;
    }

    public double adjustLimelightFlapAngle(double limelightDistance) {
        if (limelightDistance <= CLOSEDISTANCE) {
            desiredFlapAngle = CLOSE_ANGLE;
            startFlapAngle = currentFlapAngle;
            startFlapEncoder = flapEncoder.get();
            desiredFlapEncoder = startFlapEncoder + desiredFlapAngle;
            flapUp();

        } else if (limelightDistance <= MIDDISTANCE) {
            desiredFlapAngle = MID_ANGLE;
            startFlapAngle = currentFlapAngle;
            startFlapEncoder = flapEncoder.get();
            desiredFlapEncoder = startFlapEncoder + desiredFlapAngle;

            flapMid();

        } else if (limelightDistance <= FARDISTANCE) {
            desiredFlapAngle = FAR_ANGLE;
            startFlapAngle = currentFlapAngle;
            startFlapEncoder = flapEncoder.get();
            desiredFlapEncoder = startFlapEncoder + desiredFlapAngle;

            flapDown();
        }
        return desiredFlapAngle;
    }

    public void configLimelight() {
        // Forces led on
        table.getEntry("ledMode").setNumber(3);
        // Sets limelight's current pipeline to 0
        table.getEntry("pipeline").setNumber(0);
        // Sets the mode of the camera to vision processor mode
        table.getEntry("camMode").setNumber(0);
        // Defaults Limelight's snapshotting feature to off
        table.getEntry("snapshot").setNumber(0);
    }

    /*
     * public void hoodResetMethod() {
     * // Sets the max period to be stopped for 0.25 seconds
     * hoodEncoder.setMaxPeriod(0.25);
     * 
     * // Sets the motor to begin rotating until it stops
     * while (!hoodEncoder.getStopped()) {
     * hoodMotor.set(-0.25);
     * }
     * // Resets the encoder once the motor has stopped
     * while (hoodEncoder.getStopped()) {
     * hoodMotor.set(0);
     * hoodEncoder.reset();
     * currentHoodAngle = 0;
     * }
     * }
     */

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        flapEncoder.reset();
        articulatingEncoder.reset();
        extendingEncoder.reset();
    }

    double startFlapAngle = 0;
    double startFlapEncoder = 0;
    double desiredFlapEncoder = 0;

    public void flapUp() {
        if (flapEncoder.get() < (CLOSE_ANGLE - FLAP_DEADZONE)) {
            hoodMotor.set(0.5);
            currentFlapAngle = startFlapAngle + flapEncoder.get() - startFlapEncoder;
        } else if (flapEncoder.get() > (CLOSE_ANGLE + FLAP_DEADZONE)) {
            hoodMotor.set(-0.5);
            currentFlapAngle = startFlapAngle + startFlapEncoder - flapEncoder.get();
        } else {
            hoodMotor.set(0);
            currentFlapAngle = 0;
        }
    }

    public void flapMid() {
        if (flapEncoder.get() < (MID_ANGLE - FLAP_DEADZONE)) {
            hoodMotor.set(0.5);
            currentFlapAngle = startFlapAngle + flapEncoder.get() - startFlapEncoder;
        } else if (flapEncoder.get() > (MID_ANGLE + FLAP_DEADZONE)) {
            hoodMotor.set(-0.5);
            currentFlapAngle = startFlapAngle + startFlapEncoder - flapEncoder.get();
        }
    }

    public void flapDown() {
        if (flapEncoder.get() < (FAR_ANGLE - FLAP_DEADZONE)) {
            hoodMotor.set(0.5);
            currentFlapAngle = startFlapAngle + flapEncoder.get() - startFlapEncoder;
        } else if (flapEncoder.get() > (FAR_ANGLE + FLAP_DEADZONE)) {
            hoodMotor.set(-0.5);
            currentFlapAngle = startFlapAngle + startFlapEncoder - flapEncoder.get();
        }
    }
}