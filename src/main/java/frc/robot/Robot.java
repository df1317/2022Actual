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

import java.nio.file.attribute.FileStoreAttributeView;
import java.text.CollationElementIterator;
import java.util.List;
import java.util.PrimitiveIterator;

import javax.tools.ForwardingFileObject;
import javax.xml.transform.OutputKeys;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.databind.introspect.DefaultAccessorNamingStrategy.FirstCharBasedValidator;

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
import edu.wpi.first.cameraserver.CameraServer;
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
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

    // Constants
    private static final double DEADZONE = 0.1;
    private static final double SPEED_MOD = 1;
    private static final int TRIGGER = 1;
    private static final int THUMBBUTTON = 2;

    private static final double TOPCOLLECTORSPEED = 0.4;
    private static final double BOTTOMCOLLECTORSPEED = 0.8;
    private static final double EXTCLIMBERSPEED = 0.75;
    private static final double ARTCLIMBERSPEED = 0.25;

    public static final double CLOSEDISTANCE = 114;
    public static final double MIDDISTANCE = 174;
    public static final double FARDISTANCE = 228;

    // Flap constants TODO: remove me
    public static final double CLOSE_ANGLE = 0.0;
    public static final double MID_ANGLE = 5.158;
    public static final double FAR_ANGLE = 16.453;
    public static final double FLAP_DEADZONE = 0.01;
    double startFlapAngle = 0;
    double startFlapEncoder = 0;
    double desiredFlapEncoder = 0;

    // FlapFlap constants
    public static int FLAPUPTOMID = 140000;
    public static int FLAPMIDTODOWN = 50000;
    public int LIMELIGHTFLAP = UP; // limelight's desired flap position
    public int CURRENTFLAPPOSITION = UP; // actual position of flap
    public int CURRENTFLAPENCODER = 0;
    public static final int UP = 1; // LIMELIGHTFLAP desired position
    public static final int MID = 2; // LIMELIGHTFLAP desired position
    public static final int DOWN = 3; // LIMELIGHTFLAP desired position

    // PID Coefficients: don't touch
    public static final double PID_P = .006; // was 0.01, better at .009
    public static final double PID_I = 0;
    public static final double PID_D = 0;
    public static final double PID_IZ = 0;
    public static final double PID_FF = 0.00011; // was 0.0, better at 0.00001
    public static final double PID_MAXOUTPUT = 1;
    public static final double PID_MINOUTPUT = -1;

    public boolean dontShoot = false;

    // Joysticks
    private final Joystick joyE = new Joystick(0);
    private final Joystick joyL = new Joystick(1);
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
    private final WPI_VictorSPX flapMotor = new WPI_VictorSPX(8);
    private final CANSparkMax shooterMotor = new CANSparkMax(1, MotorType.kBrushless);

    private final SparkMaxPIDController shooterPID = shooterMotor.getPIDController();

    private final Counter flapEncoder = new Counter(new DigitalInput(9));
    private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private final Encoder articulatingEncoder = new Encoder(7, 8);
    private final Encoder extendingEncoder = new Encoder(5, 6);
    private final Encoder leftEncoder = new Encoder(1, 2, true); // inverted
    private final Encoder rightEncoder = new Encoder(3, 4, false); // not inverted

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

    // Limelight
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double limelightKP = -0.1;
    double limelightMinCommand = 0.1;
    double limelightLeftSteer = 0.0;
    double limelightRightSteer = 0.0;

    // Autonomous Variables
    Timer autoTimer = new Timer();
    double currentAutoTime = 0;
    boolean okayToShoot = false;

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture();

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

        SmartDashboard.putNumber("FlapUpToMid", FLAPUPTOMID);
        SmartDashboard.putNumber("FlapMidToDown", FLAPMIDTODOWN);

        resetEncoders();
    }

    public void autonomousInit() {
        autoTimer.reset();
        autoTimer.start();
        okayToShoot = false;
    }

    public void autonomousPeriodic() {
        currentAutoTime = autoTimer.get();
        double limelightTX = table.getEntry("tx").getDouble(0);
        double limelightTY = table.getEntry("ty").getDouble(0);
        double limelightTV = table.getEntry("tv").getDouble(0);
        boolean limelightVALID = limelightTV > 0;

        SmartDashboard.putBoolean("AUTO TARGET?", limelightVALID);

        double time1 = 4;
        double time2 = time1 + 5;
        double time3 = time2 + 3;
        double time4 = time3 + 3;

        // Drives forward while running the collector
        if (currentAutoTime > 0 && currentAutoTime < time1) {
            robotDrive.tankDrive(-0.5, -0.5);
            topCollectorMotor.set(-TOPCOLLECTORSPEED);
            bottomCollectorMotor.set(BOTTOMCOLLECTORSPEED);
        }
        // Turns 90 degrees clockwise
        if (currentAutoTime > time1 && currentAutoTime < time2 && !limelightVALID) {
            robotDrive.tankDrive(-0.5, 0.5);
            topCollectorMotor.set(0);
            bottomCollectorMotor.set(0);
        }
        // Limelight alignment
        if (currentAutoTime > time2 && currentAutoTime < time3) {
            limelightSteeringAlign(limelightTX);
        }
        if (currentAutoTime > time3 && currentAutoTime < time4) {
            double limelightDistance = calculateLimelightDistance(limelightTY);
            double desiredShooterRPM = calculateLimelightRPM(limelightDistance);
            doNewFlap(limelightDistance);
            shooterPID.setReference(desiredShooterRPM, ControlType.kVelocity);
            okayToShoot = Math.abs(desiredShooterRPM - shooterEncoder.getVelocity()) < 75;
            if (okayToShoot) {
                bottomCollectorMotor.set(1);
                topCollectorMotor.set(1);
            }
        }

        if (currentAutoTime > time4) {
            bottomCollectorMotor.set(0);
            topCollectorMotor.set(0);
            shooterMotor.set(0);
            robotDrive.tankDrive(0, 0);
        }
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

        SmartDashboard.putNumber("Articulating Encoder", articulatingEncoder.get());
        SmartDashboard.putNumber("Extending Encoder", extendingEncoder.get());

        // Set PID Coefficients
        shooterPID.setP(PID_P);
        shooterPID.setI(PID_I);
        shooterPID.setD(PID_D);
        shooterPID.setIZone(PID_IZ);
        shooterPID.setFF(PID_FF);
        shooterPID.setOutputRange(PID_MINOUTPUT, PID_MAXOUTPUT);

        // Buttons
        boolean halfsiesL = joyL.getRawButton(THUMBBUTTON);
        boolean halfsiesR = joyR.getRawButton(THUMBBUTTON);
        boolean initialCollectionLButton = joyL.getRawButton(TRIGGER);
        boolean initialCollectionRButton = joyR.getRawButton(TRIGGER);
        boolean feederCollectionButton = joyE.getRawButton(THUMBBUTTON);
        boolean hoodAdjustForward = joyE.getRawButton(12);
        boolean hoodAdjustBackward = joyE.getRawButton(10);
        boolean ejectButtonL = joyL.getRawButton(3);
        boolean ejectButtonR = joyR.getRawButton(3);
        boolean stopFlap = joyE.getRawButton(5);
        boolean dontShootButton = joyE.getRawButton(11);

        // Limelight Buttons
        boolean limelightAlignButtonL = joyL.getRawButton(4);
        boolean limelightAlignButtonR = joyR.getRawButton(4);
        boolean limelightShootButton = joyE.getRawButton(TRIGGER);

        // Manual Climber Buttons
        boolean articulatingClimberButton = joyE.getRawButton(3);
        boolean windClimberButton = joyE.getRawButton(6);
        boolean unwindClimberButton = joyE.getRawButton(4);

        // Driving Variables
        double halfSpeed = 1;

        // Updating Variables
        double shooterSpeedManual = (joyE.getRawAxis(3) / 4) + 0.75; // converts [-1, 1] to [-1/4, 1/4] to [0.5, 1]

        // Limelight Variables
        double limelightTV = table.getEntry("tv").getDouble(0);
        boolean limelightVALID = limelightTV > 0;
        double limelightTX = table.getEntry("tx").getDouble(0);
        double limelightTY = table.getEntry("ty").getDouble(0);

        double limelightDistance = calculateLimelightDistance(limelightTY);
        double desiredShooterRPM = calculateLimelightRPM(limelightDistance);

        if (stopFlap) {
            flapMotor.set(0);
        }

        // Hood Adjustment Manual
        if (hoodAdjustForward) {
            flapMotor.set(0.5);
        } else if (hoodAdjustBackward) {
            flapMotor.set(-0.5);
        }

        // Fancy Ternary Operators heck yeah
        halfSpeed = halfsiesL || halfsiesR ? 0.75 : 1;

        // Drivetrain Controls: left and right joysticks
        if (Math.abs(joyL.getY()) > DEADZONE || Math.abs(joyR.getY()) > DEADZONE) {
            robotDrive.tankDrive(joyL.getY() * SPEED_MOD * halfSpeed, joyR.getY() * SPEED_MOD * halfSpeed);
        } else {
            robotDrive.tankDrive(0, 0);
        }

        // Limelight Alignment Controls
        if (limelightAlignButtonL || limelightAlignButtonR) {
            limelightLeftSteer = limelightSteeringAlign(limelightTX);
            limelightRightSteer = limelightSteeringAlign(limelightTX);

            robotDrive.tankDrive(limelightLeftSteer, limelightRightSteer);
        }

        // Automatic Flap Controls using Operator Trigger
        doNewFlap(limelightDistance);

        // Shooting Motor Controls
        if (limelightShootButton) {
            // Sets shooting motor PID to limelight's calculated RPM value when enabled
            shooterPID.setReference(desiredShooterRPM,
                    CANSparkMax.ControlType.kVelocity);
        } else {
            // Don't shoot toggle
            if (dontShootButton && !dontShoot) {
                dontShoot = true;
            } else if (dontShootButton && dontShoot) {
                dontShoot = false;
            }

            // Shooter motor toggle controls the robot
            if (dontShoot) {
                shooterMotor.set(0);
            } else if (!articulatingClimberButton && !windClimberButton && !unwindClimberButton) {
                shooterPID.setReference(3000, CANSparkMax.ControlType.kVelocity);
            }
        }

        boolean RPMinRange = (Math.abs(desiredShooterRPM - shooterEncoder.getVelocity()) < 75);

        // SmartDashboard RPM Values
        SmartDashboard.putBoolean("Valid Target?", limelightVALID);
        SmartDashboard.putBoolean("RPM IN RANGE", RPMinRange);
        SmartDashboard.putNumber("Limelight Distance", calculateLimelightDistance(limelightTY));
        SmartDashboard.putNumber("Limelight RPM Value", desiredShooterRPM);
        SmartDashboard.putNumber("Actual RPM", shooterEncoder.getVelocity());

        // Collection Controls
        if (initialCollectionLButton || initialCollectionRButton) {
            topCollectorMotor.set(-TOPCOLLECTORSPEED);
            bottomCollectorMotor.set(BOTTOMCOLLECTORSPEED);
        } else if (feederCollectionButton) {
            topCollectorMotor.set(TOPCOLLECTORSPEED);
            bottomCollectorMotor.set(BOTTOMCOLLECTORSPEED);
        } else if (ejectButtonL || ejectButtonR) {
            topCollectorMotor.set(-TOPCOLLECTORSPEED);
            bottomCollectorMotor.set(-BOTTOMCOLLECTORSPEED);
        } else {
            topCollectorMotor.set(0);
            bottomCollectorMotor.set(0);
        }

        // Articulating Climber Controls: operator buttons 5 & 3
        if (articulatingClimberButton) {
            articulatingClimbers.set(joyE.getY() * ARTCLIMBERSPEED * -1);
            shooterMotor.set(0);
        } else {
            articulatingClimbers.set(0);
        }
        // Extending Climber Winch Controls: operator buttons 6 & 4
        if (windClimberButton) {
            extendingClimbers.set(EXTCLIMBERSPEED);
            shooterMotor.set(0);
        } else if (unwindClimberButton) {
            extendingClimbers.set(-EXTCLIMBERSPEED);
            shooterMotor.set(0);
        } else {
            extendingClimbers.set(0);
        }

    }

    public double limelightSteeringAlign(double limelightTX) {
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

    // int climbStageFirst = 1;
    // int climbStageSecond = 1;
    // int climbStageThird = 1;

    public int curFlapPosition = UP;
    public int goalFlapPosition = UP;
    public int goalFlapEncoder = 0;

    public void newFlap() {
        if (curFlapPosition == goalFlapPosition)
            return;

        int curEncoder = flapEncoder.get();

        if (curEncoder >= goalFlapEncoder) {
            curFlapPosition = goalFlapPosition;
            flapMotor.set(0);
        } else {
            int sign = (goalFlapPosition > curFlapPosition) ? 1 : -1;
            flapMotor.set(0.5 * sign);
        }
    }

    public void setDesiredFlapPosition(int desiredPosition) {
        // Don't let goal position change if we're in the middle of movement
        if (curFlapPosition != goalFlapPosition)
            return;

        // Don't do anything if desiredPosition is same as curPosition
        if (curFlapPosition == desiredPosition)
            return;

        // Don't allow an invalid goal position
        if (desiredPosition != UP && desiredPosition != MID && desiredPosition != DOWN)
            return;

        int encoderDistance = 0;

        // Figure out the distance the encoder must move to get from curFlapPosition to
        // desiredPosition
        if (curFlapPosition == UP) {
            if (desiredPosition == MID) {
                encoderDistance = FLAPUPTOMID;
            } else if (desiredPosition == DOWN) {
                encoderDistance = (FLAPUPTOMID + FLAPMIDTODOWN);
            }
        } else if (curFlapPosition == MID) {
            if (desiredPosition == UP) {
                encoderDistance = FLAPUPTOMID;
            } else if (desiredPosition == DOWN) {
                encoderDistance = FLAPMIDTODOWN;
            }
        } else if (curFlapPosition == DOWN) {
            if (desiredPosition == UP) {
                encoderDistance = (FLAPUPTOMID + FLAPMIDTODOWN) + 5000;
            } else if (desiredPosition == MID) {
                encoderDistance = FLAPMIDTODOWN;
            }
        }

        // Sanity check, shouldn't occur
        if (encoderDistance == 0)
            return;

        goalFlapPosition = desiredPosition;
        goalFlapEncoder = flapEncoder.get() + encoderDistance;
    }

    // boolean flapToggleButtonLastPressed = false;

    public void doNewFlap(double limelightDistance) {
        if (limelightDistance <= CLOSEDISTANCE) {
            setDesiredFlapPosition(UP);
        } else if (limelightDistance <= MIDDISTANCE) {
            setDesiredFlapPosition(MID);
        } else if (limelightDistance <= FARDISTANCE) {
            setDesiredFlapPosition(DOWN);
        }

        SmartDashboard.putBoolean("FLAP SUCCESSFUL", curFlapPosition == goalFlapPosition);
        SmartDashboard.putNumber("LIMELIGHTFLAP", goalFlapPosition);
        SmartDashboard.putNumber("CURRENTFLAPPOSITION", curFlapPosition);
        SmartDashboard.putNumber("CURRENTFLAPENCODER", flapEncoder.get());
        /*
         * if (joyL.getRawButton(5) && !flapToggleButtonLastPressed) {
         * int pos = curFlapPosition;
         * if (++pos > DOWN) {
         * pos = UP;
         * }
         * 
         * setDesiredFlapPosition(pos);
         * }
         */
        // flapToggleButtonLastPressed = joyL.getRawButton(5);

        // Move flap to goal position no matter what
        newFlap();
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

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        flapEncoder.reset();
        articulatingEncoder.reset();
        extendingEncoder.reset();
    }

}