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
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;

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

    private static final double CLOSEDISTANCE = 114;
    private static final double MIDDISTANCE = 174;
    private static final double FARDISTANCE = 228;

    private static final double SET_CLOSE_ANGLE = 0.0;
    private static final double SET_MID_ANGLE = .000002;
    private static final double SET_FAR_ANGLE = .000004;

    private static final double CHASSISWHEELWIDTH = 20; // inches

    // PID Coefficients
    // Don't touch
    private static final double PID_P = .009; // was 0.01, better at .009
    private static final double PID_I = 0;
    private static final double PID_D = 0;
    private static final double PID_IZ = 0;
    private static final double PID_FF = 0.00001; // was 0.0, better at 0.00001
    private static final double PID_MAXOUTPUT = 1;
    private static final double PID_MINOUTPUT = -1;

    // Joysticks
    private final Joystick joyE = new Joystick(0);
    private final Joystick joyL = new Joystick(2);
    private final Joystick joyR = new Joystick(1);

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

    private final Counter hoodEncoder = new Counter(new DigitalInput(9));
    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private final Encoder articulatingEncoder = new Encoder(5, 6);
    private final Encoder extendingEncoder = new Encoder(7, 8);
    private final Encoder leftEncoder = new Encoder(1, 2, false); // not inverted
    private final Encoder rightEncoder = new Encoder(3, 4, true); // inverted

    // Gyroscope
    AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Motor Controller Groups
    private final MotorControllerGroup leftMotorGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    private final MotorControllerGroup rightMotorGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);
    private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    private final MotorControllerGroup extendingClimbers = new MotorControllerGroup(topExtendingClimber,
            bottomExtendingClimber);
    private final MotorControllerGroup articulatingClimbers = new MotorControllerGroup(topArticulatingClimber,
            bottomArticulatingClimber);

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    // Limelight
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double limelightKP = -0.1;
    double limelightMinCommand = 0.1;
    double limelightLeftSteer = 0.0;
    double limelightRightSteer = 0.0;

    // Hood Values
    double desiredHoodAngle = SET_MID_ANGLE;
    double currentHoodAngle = 0.0;

    // Climber: obsolete, remove me
    double firstButtonTime = 0.0;
    double secondButtonTime = 0.0;
    double thirdButtonTime = 0.0;
    double teleopStartTime = 0.0;

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

        // Set PID Coefficients to SmartDashboard
        // SmartDashboard.putNumber("P Gain", PID_P);
        // SmartDashboard.putNumber("I Gain", PID_I);
        // SmartDashboard.putNumber("D Gain", PID_D);
        // SmartDashboard.putNumber("I Zone", PID_IZ);
        // SmartDashboard.putNumber("Feed Forward", PID_FF);
        // SmartDashboard.putNumber("Max Output", PID_MAXOUTPUT);
        // SmartDashboard.putNumber("Min Output", PID_MINOUTPUT);

        // Resets encoders everytime the robot is powered on
        hoodEncoder.reset();
        articulatingEncoder.reset();
        extendingEncoder.reset();

        leftEncoder.setDistancePerPulse(10);
        rightEncoder.setDistancePerPulse(10);

        resetEncoders();
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void arcadeDrive(double fwd, double rot) {
        robotDrive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotorGroup.setVoltage(leftVolts);
        rightMotorGroup.setVoltage(rightVolts);
        robotDrive.feed();
    }

    public double getAverageEncoderDistance = (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;

    public void setMaxOutput(double maxOutput) {
        robotDrive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    double getHeading = gyro.getRotation2d().getDegrees();
    double getTurnRate = -gyro.getRate();

    @Override
    public void teleopInit() {

        // How long the robot has been turned on when teleop is first enabled
        // TODO: remove me
        double teleopStartTime = Timer.getFPGATimestamp();

    }

    @Override
    public void teleopPeriodic() {

        SmartDashboard.putNumber("Flap Encoder", (currentHoodAngle / (1000000 * 360)));
        SmartDashboard.putNumber("Articulating Encoder", articulatingEncoder.getDistance());
        SmartDashboard.putNumber("Extending Encoder", extendingEncoder.getDistance());

        // Set PID Coefficients
        shooterPID.setP(PID_P);
        shooterPID.setI(PID_I);
        shooterPID.setD(PID_D);
        shooterPID.setIZone(PID_IZ);
        shooterPID.setFF(PID_FF);
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
            currentHoodAngle += hoodEncoder.get();
        } else if (hoodAdjustBackward) {
            hoodMotor.set(-0.5);
            currentHoodAngle -= hoodEncoder.get();
        } else {
            hoodMotor.set(0);
        }

        // Hood Encoder Reset Manual? Except it doesn't always seem to work
        if (currentHoodAngle < 0) {
            currentHoodAngle = 0;
        }

        // Fancy Ternary Operators heck yeah
        leftHalfSpeed = halfsiesL ? 0.5 : 0;
        rightHalfSpeed = halfsiesR ? 0.5 : 0;

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

        // Shooting Motor Controls
        if (limelightShootButton) {
            // Sets shooting motor PID to limelight's calculated RPM value when enabled
            shooterPID.setReference(desiredShooterRPM,
                    CANSparkMax.ControlType.kVelocity);
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

        // Articulating Climber: set at an angle (not vertical) to grab bar
        // Extending Climber: starts fully extended to pull us onto bar
        if (firstButton) {
            if (extendingEncoder.getDistance() > EXTPULLUP) {
                extendingClimbers.set(-EXTCLIMBERSPEED);
            } else {
                extendingClimbers.set(0);
            }

            if (extendingEncoder.getDistance() > (EXTPULLUP - 20) && articulatingEncoder.getDistance() < 0) {
                articulatingClimbers.set(ARTCLIMBERSPEED);
            } else {
                articulatingClimbers.set(0);
            }
        }

        if (secondButton) {
            while (articulatingEncoder.getDistance() < ARTROTATETOGRAB) {
                articulatingClimbers.set(ARTCLIMBERSPEED);
            }
            while (articulatingEncoder.getDistance() < (ARTROTATETOGRAB - 15)) {
                extendingClimbers.set(EXTCLIMBERSPEED);
            }

        }

        // Articulating Climber Controls [MANUAL]: operator buttons 5 & 3
        if (articulatingClimberButton) {
            articulatingClimbers.set(ARTCLIMBERSPEED);
        } else if (articulatingClimberOtherwayButton) {
            articulatingClimbers.set(-ARTCLIMBERSPEED);
        } else {
            articulatingClimbers.set(0);
        }

        // Extending Climber Winch Controls [MANUAL]: operator buttons 6 & 4
        if (windClimberButton) {
            extendingClimbers.set(EXTCLIMBERSPEED);
        } else if (unwindClimberButton) {
            extendingClimbers.set(-EXTCLIMBERSPEED);
        } else {
            extendingClimbers.set(0);
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

    // TODO: set up method for adjust hood according to limelight distance
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

    public double calculateLimelightHoodAngle(double limelightDistance) {
        if (limelightDistance <= CLOSEDISTANCE) {
            desiredHoodAngle = SET_CLOSE_ANGLE;
        } else if (limelightDistance <= MIDDISTANCE) {
            desiredHoodAngle = SET_MID_ANGLE;
        } else if (limelightDistance <= FARDISTANCE) {
            desiredHoodAngle = SET_FAR_ANGLE;
        } else {
            desiredHoodAngle = SET_MID_ANGLE;
        }
        return desiredHoodAngle;
    }

    private void adjustLimelightHoodAngle() {
        if (currentHoodAngle < desiredHoodAngle) {
            hoodMotor.set(0.25);
            currentHoodAngle += hoodEncoder.get();
        } else if (currentHoodAngle > desiredHoodAngle) {
            hoodMotor.set(-0.25);
            currentHoodAngle -= hoodEncoder.get();
        } else {
            hoodMotor.set(0);
        }
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
        hoodEncoder.reset();
        articulatingEncoder.reset();
        extendingEncoder.reset();
    }
}