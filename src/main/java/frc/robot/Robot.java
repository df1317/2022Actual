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
import java.util.List;
import java.util.PrimitiveIterator;

import javax.tools.ForwardingFileObject;

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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

    // Constants
    private static final double articulatingClimberSpeed = 0.5;
    private static final double extendingClimberSpeed = 0.5;

    private static final double EXTPULLUP = 0;
    private static final double ARTROTATETOGRAB = 0;

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

    // Flap constants
    public static final double CLOSE_ANGLE = 0.0;
    public static final double MID_ANGLE = 5.158;
    public static final double FAR_ANGLE = 16.453;
    public static final double FLAP_DEADZONE = 0.01;

    double startFlapAngle = 0;
    double startFlapEncoder = 0;
    double desiredFlapEncoder = 0;

    // FlapFlap constants
    public static final double FLAPUPTOMID = 6;
    public static final double FLAPMIDTODOWN = 8;
    public static final double FLAPUPTODOWN = 14;
    public double LIMELIGHTFLAP = UP; // limelight's desired flap position
    public double CURRENTFLAPPOSITION = UP; // actual position of flap
    public double CURRENTFLAPENCODER = 0;
    public static final double UP = 1; // LIMELIGHTFLAP desired position
    public static final double MID = 2; // LIMELIGHTFLAP desired position
    public static final double DOWN = 3; // LIMELIGHTFLAP desired position

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
    private final WPI_VictorSPX flapMotor = new WPI_VictorSPX(8);
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
    boolean movingFlap = false;
    boolean flapAdjusted = false;
    double limelightFlapAngle = 0;
    double actualFlapAngle = 0;

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
        SmartDashboard.putNumber("Flap Encoder", flapEncoder.get());

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
        boolean resetHoodDegreesTest = joyL.getRawButton(6);
        boolean stopFlap = joyE.getRawButton(5);

        // Limelight Buttons
        boolean limelightAlignButtonL = joyL.getRawButton(4);
        boolean limelightAlignButtonR = joyR.getRawButton(4);
        boolean limelightShootButton = joyE.getRawButton(TRIGGER);

        // Manual Climber Buttons
        boolean articulatingClimberButton = joyE.getRawButton(3);
        boolean windClimberButton = joyE.getRawButton(6);
        boolean unwindClimberButton = joyE.getRawButton(4);

        // Automated Climber Buttons
        boolean firstButton = joyE.getRawButton(11);
        boolean secondButton = joyE.getRawButton(8);
        boolean thirdButton = joyE.getRawButton(7);

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
        boolean limelightHasTarget = limelightTV > 0;

        SmartDashboard.putBoolean("Valid Target?", limelightHasTarget);

        if (stopFlap) {
            flapMotor.set(0);
        }

        // Hood Adjustment Manual
        if (hoodAdjustForward) {
            flapMotor.set(0.5);
            // currentFlapAngle += flapEncoder.get();
            movingFlap = false;
        } else if (hoodAdjustBackward) {
            flapMotor.set(-0.5);
            // currentFlapAngle -= flapEncoder.get();
            movingFlap = false;
        } else {
            if (!movingFlap) {
                flapMotor.set(0);
            }
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
            limelightLeftSteer = limelightSteeringAlign(limelightAlignButtonL, limelightAlignButtonR, limelightTX);
            limelightRightSteer = limelightSteeringAlign(limelightAlignButtonL, limelightAlignButtonR, limelightTX);

            robotDrive.tankDrive(limelightLeftSteer, limelightRightSteer);
        }

        // Shooting Motor Controls
        if (limelightShootButton) {
            // Sets shooting motor PID to limelight's calculated RPM value when enabled
            shooterPID.setReference(desiredShooterRPM,
                    CANSparkMax.ControlType.kVelocity);
            flapAngleAdjustment(limelightDistance);
            if (!flapAdjusted) {
                // adjustLimelightFlapAngle(limelightDistance);
                flapAdjusted = true;
            }
        } else {
            if (!articulatingClimberButton && !windClimberButton && !unwindClimberButton) {
                shooterPID.setReference(3000, CANSparkMax.ControlType.kVelocity);
            }
            flapAdjusted = false;
        }

        boolean RPMinRange = (Math.abs(desiredShooterRPM - shooterEncoder.getVelocity()) < 75);

        // SmartDashboard Values for testing
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

        // Articulating Climber Controls [MANUAL]: operator buttons 5 & 3
        if (articulatingClimberButton) {
            articulatingClimbers.set(joyE.getY() * ARTCLIMBERSPEED);
            shooterMotor.set(0);
        } else {
            articulatingClimbers.set(0);
        }
        // Extending Climber Winch Controls [MANUAL]: operator buttons 6 & 4
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

    public void flapFlap(double limelightDistance) {
        if (limelightDistance <= CLOSEDISTANCE) {
            LIMELIGHTFLAP = UP;
        } else if (limelightDistance <= MIDDISTANCE) {
            LIMELIGHTFLAP = MID;
        } else if (limelightDistance <= FARDISTANCE) {
            LIMELIGHTFLAP = DOWN;
        }

        SmartDashboard.putBoolean("FLAP", CURRENTFLAPPOSITION == LIMELIGHTFLAP);

        if (CURRENTFLAPPOSITION != LIMELIGHTFLAP) {
            // Gets the current encoder value at the start of adjustment
            CURRENTFLAPENCODER = flapEncoder.get();
            SmartDashboard.putNumber("LIMELIGHTFLAP", LIMELIGHTFLAP);
            SmartDashboard.putNumber("CURRENTFLAPPOSITION", CURRENTFLAPPOSITION);
            SmartDashboard.putNumber("CURRENTFLAPENCODER", CURRENTFLAPENCODER);

            // UP
            if (LIMELIGHTFLAP == UP) {
                if (CURRENTFLAPPOSITION == MID) {
                    if (flapEncoder.get() < FLAPUPTOMID + CURRENTFLAPENCODER) {
                        flapMotor.set(-0.5);
                    } else {
                        flapMotor.set(0);
                        CURRENTFLAPPOSITION = UP;
                    }
                } else if (CURRENTFLAPPOSITION == DOWN) {
                    if (flapEncoder.get() < FLAPUPTODOWN + CURRENTFLAPENCODER) {
                        flapMotor.set(-0.5);
                    } else {
                        flapMotor.set(0);
                        CURRENTFLAPPOSITION = UP;
                    }
                }
            }

            // MID
            if (LIMELIGHTFLAP == MID) {
                if (CURRENTFLAPPOSITION == UP) {
                    if (flapEncoder.get() < FLAPUPTOMID + CURRENTFLAPENCODER) {
                        flapMotor.set(0.5);
                    } else {
                        flapMotor.set(0);
                        CURRENTFLAPPOSITION = MID;
                    }
                } else if (CURRENTFLAPPOSITION == DOWN) {
                    if (flapEncoder.get() < FLAPMIDTODOWN + CURRENTFLAPENCODER) {
                        flapMotor.set(-0.5);
                    } else {
                        flapMotor.set(0);

                    }
                }
            }

            // DOWN
            if (LIMELIGHTFLAP == DOWN) {
                if (CURRENTFLAPPOSITION == UP) {
                    if (flapEncoder.get() < FLAPUPTODOWN + CURRENTFLAPENCODER) {
                        flapMotor.set(0.5);
                    } else {
                        flapMotor.set(0);
                        CURRENTFLAPPOSITION = DOWN;
                    }
                } else if (CURRENTFLAPPOSITION == MID) {
                    if (flapEncoder.get() < FLAPMIDTODOWN + CURRENTFLAPENCODER) {
                        flapMotor.set(-0.5);
                    } else {
                        flapMotor.set(0);
                        CURRENTFLAPPOSITION = DOWN;
                    }
                }
            }
        }

    }

    public void flapAngleAdjustment(double limelightDistance) {
        boolean flapDirection = flapEncoder.getDirection();
        double flapGet = flapEncoder.get();

        if (flapDirection && (flapGet != actualFlapAngle)) {
            actualFlapAngle += flapEncoder.get();
        } else if (!flapDirection && (flapGet != actualFlapAngle)) {
            actualFlapAngle -= flapEncoder.get();
        }

        SmartDashboard.putNumber("ActualFlapAngle", actualFlapAngle);
        SmartDashboard.putBoolean("FlapDirection", flapDirection);
        SmartDashboard.putNumber("FlapAngle", limelightFlapAngle);

        if (limelightDistance <= CLOSEDISTANCE) {
            limelightFlapAngle = CLOSE_ANGLE;
        } else if (limelightDistance <= MIDDISTANCE) {
            limelightFlapAngle = MID_ANGLE;
        } else if (limelightDistance <= FARDISTANCE) {
            limelightFlapAngle = FAR_ANGLE;
        }

        if (limelightFlapAngle > actualFlapAngle) {
            flapMotor.set(0.5);
        } else if (limelightFlapAngle < actualFlapAngle) {
            flapMotor.set(-0.5);
        } else {
            flapMotor.set(0);
        }

    }

    public void adjustLimelightFlapAngle(double limelightDistance) {
        if (movingFlap) {
            flapMotion();
        } else {
            startFlapAngle = currentFlapAngle;
            startFlapEncoder = flapEncoder.get();
            int flapDirection = 1;

            if (limelightDistance <= CLOSEDISTANCE) {
                desiredFlapAngle = CLOSE_ANGLE;
            } else if (limelightDistance <= MIDDISTANCE) {
                desiredFlapAngle = MID_ANGLE;
            } else if (limelightDistance <= FARDISTANCE) {
                desiredFlapAngle = FAR_ANGLE;
            } else {
                System.out.println("NO DESIRED ANGLE");
            }
            System.out.println("desired angle: " + desiredFlapAngle);
            desiredFlapEncoder = startFlapEncoder + Math.abs(startFlapAngle -
                    desiredFlapAngle);
            if (startFlapAngle > desiredFlapAngle) {
                flapDirection = -1;
            } else {
                flapDirection = 1;
            }
            System.out.println("setting desired flap encoder to " + desiredFlapEncoder);
            flapMotor.set(0.5 * flapDirection);
            movingFlap = true;

            SmartDashboard.putNumber("Current ANGLE", currentFlapAngle);
            SmartDashboard.putNumber("Desired ANGLE", desiredFlapAngle);
            SmartDashboard.putNumber("Start ENCODER", startFlapEncoder);
            SmartDashboard.putNumber("Desired ENCODER", desiredFlapEncoder);
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

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        flapEncoder.reset();
        articulatingEncoder.reset();
        extendingEncoder.reset();
    }

    public void flapMotion() {
        System.out.println("current: " + flapEncoder.get() + "; desired: " + desiredFlapEncoder);
        if (flapEncoder.get() >= desiredFlapEncoder) {
            flapMotor.set(0);
            movingFlap = false;
            System.out.println("stopping hood motion");
            currentFlapAngle = desiredFlapAngle;
        }
        System.out.println("flap encoder < desired");
    }

}