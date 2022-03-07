// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODO: 
implement new limelight code
automate climber for real
    button pressed/released issues?
    hold button down instead?
    no time requirement for next stage of climbing?
    FPGAtimer situation not working? other timing options?
        look into wpilibj.TImer import for timing options
        previous robot code
50% drivetrain speed - halfSpeedButton(L/R)
clean up code
? hood motor set up 
   ? add to limelight code
*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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
import edu.wpi.first.networktables.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Robot extends TimedRobot {

    // Constants
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

    // PID Coefficients
    private static final double PID_P = 6e-5;
    private static final double PID_I = 0;
    private static final double PID_D = 0;
    private static final double PID_IZ = 0;
    private static final double PID_FF = 0.000015;
    private static final double PID_MAXOUTPUT = 1;
    private static final double PID_MINOUTPUT = -1;
    private static final double PID_MAXRPM = 5700;

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
    private final RelativeEncoder shooterRPM = shooterMotor.getEncoder();
    private final SparkMaxPIDController shooterPID = shooterMotor.getPIDController();

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
    double limelightMinCommand = 0.3;
    double limelightLeftSteer = 0.0;
    double limelightRightSteer = 0.0;
    double limelightShootingRPM = 0.0;

    // Climber: obsolete
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

        // Set PID Coefficients to SmartDashboard, referenced in teleop code
        SmartDashboard.putNumber("P Gain", PID_P);
        SmartDashboard.putNumber("I Gain", PID_I);
        SmartDashboard.putNumber("D Gain", PID_D);
        SmartDashboard.putNumber("I Zone", PID_IZ);
        SmartDashboard.putNumber("Feed Forward", PID_FF);
        SmartDashboard.putNumber("Max Output", PID_MAXOUTPUT);
        SmartDashboard.putNumber("Min Output", PID_MINOUTPUT);
    }

    @Override
    public void teleopInit() {

        // How long the robot has been turned on when teleop is first enabled
        // TODO: remove me
        double teleopStartTime = Timer.getFPGATimestamp();

    }

    @Override
    public void teleopPeriodic() {

        // Set PID Coefficients: is this necessary since robotInit already set them?
        shooterPID.setP(PID_P);
        shooterPID.setI(PID_I);
        shooterPID.setD(PID_D);
        shooterPID.setIZone(PID_IZ);
        shooterPID.setFF(PID_FF);
        shooterPID.setOutputRange(PID_MINOUTPUT, PID_MAXOUTPUT);

        // Buttons
        boolean spinShooterButton = joyE.getRawButton(TRIGGER);
        boolean halfsiesL = joyL.getRawButton(TRIGGER);
        boolean halfsiesR = joyR.getRawButton(TRIGGER);
        boolean initialCollectionLButton = joyL.getRawButton(THUMBBUTTON);
        boolean initialCollectionRButton = joyR.getRawButton(THUMBBUTTON);
        boolean feederCollectionButton = joyE.getRawButton(THUMBBUTTON);
        boolean hoodAdjustForward = joyE.getRawButton(12);
        boolean hoodAdjustBackward = joyE.getRawButton(10);
        boolean ejectButtonL = joyL.getRawButton(3);
        boolean ejectButtonR = joyR.getRawButton(3);

        // Limelight Buttons
        boolean limelightAlignButtonL = joyL.getTop();
        boolean limelightAlignButtonR = joyR.getTop();
        boolean limelightShoot = joyE.getTop();

        // Manual Climber Buttons
        boolean articulatingClimberButton = joyE.getRawButton(5);
        boolean articulatingClimberOtherwayButton = joyE.getRawButton(3);
        boolean windClimberButton = joyE.getRawButton(6);
        boolean unwindClimberButton = joyE.getRawButton(4);

        // Automated Climber Buttons: obsolete
        boolean firstButtonPressed = joyE.getRawButtonPressed(11);
        boolean secondButtonPressed = joyE.getRawButtonPressed(8);
        boolean thirdButtonPressed = joyE.getRawButtonPressed(7);

        // Updating Variables
        double shooterSpeedManual = (joyE.getRawAxis(3) / 4) + 0.75; // converts [-1, 1] to [-1/4, 1/4] to [0.5, 1]

        // Limelight Variables
        double limelightTV = table.getEntry("tv").getDouble(0);
        double limelightTA = table.getEntry("ta").getDouble(0);
        double limelightTX = table.getEntry("tx").getDouble(0);
        double limelightTY = table.getEntry("ty").getDouble(0);
        double limelightDistance = calculateLimelightDistance(limelightTY);
        SmartDashboard.putNumber("Valid Target?", limelightTV);
        SmartDashboard.putNumber("Limelight tx", limelightTX);
        SmartDashboard.putNumber("Limelight ty", limelightTY);
        SmartDashboard.putNumber("Limelight ta", limelightTA);

        calculateLimelightDistance(limelightTY);
        calculateLimelightShootingSpeed(limelightDistance);

        // Hood Adjustment Manual
        // TODO: hood adjustment with limelight distances
        if (hoodAdjustForward) {
            hoodMotor.set(0.5);
        } else if (hoodAdjustBackward) {
            hoodMotor.set(-0.5);
        } else {
            hoodMotor.set(0);
        }

        // Drivetrain Controls: left and right joysticks
        if (Math.abs(joyL.getY()) > DEADZONE || Math.abs(joyR.getY()) > DEADZONE) {
            if (halfsiesL) {
                robotDrive.tankDrive(joyL.getY() * SPEED_MOD * 0.5, joyR.getY() * SPEED_MOD);
            } else if (halfsiesR) {
                robotDrive.tankDrive(joyL.getY() * SPEED_MOD, joyR.getY() * SPEED_MOD * 0.5);
            } else {
                robotDrive.tankDrive(joyL.getY() * SPEED_MOD, joyR.getY() * SPEED_MOD);
            }
        } else {
            robotDrive.tankDrive(0, 0);
        }

        // Limelight Alignment Controls
        if (limelightAlignButtonL || limelightAlignButtonR) {
            limelightSteeringAlign(limelightAlignButtonL, limelightAlignButtonR, limelightTX);
            robotDrive.tankDrive(limelightLeftSteer, limelightRightSteer);
        }

        // Shooting Motor Controls
        if (limelightShoot) {
            // Sets shooting motor PID to limelight's calculated RPM value when enabled
            shooterPID.setReference(calculateLimelightShootingSpeed(limelightDistance),
                    CANSparkMax.ControlType.kVelocity);
        } else if (spinShooterButton) {
            // Else uses value from operator control
            shooterMotor.set(shooterSpeedManual);
        } else {
            shooterMotor.set(0);
        }

        // SmartDashboard Values for testing
        SmartDashboard.putNumber("Limelight Distance, bumper to target", calculateLimelightDistance(limelightTY));
        SmartDashboard.putNumber("Limelight RPM Value", calculateLimelightShootingSpeed(limelightDistance));
        SmartDashboard.putNumber("Actual RPM", shooterRPM.getVelocity());
        SmartDashboard.putBoolean("Limelight Shooting Enabled?", limelightShoot);
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

        // Automated Climber Controls: buttons 11, 8, 7
        // yikes help
        /*
         * Everything is timed to be 1 second apart. Extending arms are lifted slightly
         * before they are rotated off of a bar.
         * -ExtendingClimberSpeed = winch pulls up robot
         * +ExtendingClimberSpeed = releases winch to allow extension of arms
         * -ArticulatingClimberSpeed = causes clockwise rotation |/
         * +ArticulatingClimberSpeed = causes counterclockwise rotation \|
         */
        if (firstButtonPressed) {
            // Button 11 is pressed to pull robot up onto bar 1
            firstButtonTime = Timer.getFPGATimestamp() - teleopStartTime;
            extendingClimbers.set(-EXTCLIMBERSPEED);
            articulatingClimbers.set(0);
        } else if (firstButtonTime > 2 && secondButtonPressed) {
            secondButtonTime = Timer.getFPGATimestamp() - teleopStartTime;

            if (secondButtonTime <= 1) {
                // Button 8 is pressed to slightly raise extending arms off of bar 1
                // Articulating arms are rotated to angle towards bar 2
                topExtendingClimber.set(EXTCLIMBERSPEED * 0.5);
                bottomExtendingClimber.set(EXTCLIMBERSPEED * 0.5);
                topArticulatingClimber.set(-ARTCLIMBERSPEED);
                bottomArticulatingClimber.set(-ARTCLIMBERSPEED);
            } else if (secondButtonTime > 1 && secondButtonTime <= 2) {
                // After rotating, we extend to reach for bar 2
                topExtendingClimber.set(EXTCLIMBERSPEED);
                bottomExtendingClimber.set(EXTCLIMBERSPEED);
                topArticulatingClimber.set(0);
                bottomArticulatingClimber.set(0);
            } else if (secondButtonTime > 3 && secondButtonTime <= 4) {
                // After reaching towards bar 2, we rotate to allow extending arms to grasp it
                topExtendingClimber.set(0);
                bottomExtendingClimber.set(0);
                topArticulatingClimber.set(ARTCLIMBERSPEED);
                bottomArticulatingClimber.set(ARTCLIMBERSPEED);
            }
        } else if (secondButtonTime > 3 && thirdButtonPressed) {
            thirdButtonTime = Timer.getFPGATimestamp() - teleopStartTime;
            if (thirdButtonTime < 1) {
                // Button 7 is pressed to pull robot onto bar 2
                topExtendingClimber.set(-EXTCLIMBERSPEED);
                bottomExtendingClimber.set(-EXTCLIMBERSPEED);
                topArticulatingClimber.set(0);
                bottomArticulatingClimber.set(0);
            } else if (thirdButtonTime >= 1 && thirdButtonTime <= 2) {
                // Extending arms lower a little bit to allow articulating arms to sneak under
                // Articulating arms are rotated to vertical to allow us to grasp bar 2
                topExtendingClimber.set(EXTCLIMBERSPEED * 0.5);
                bottomExtendingClimber.set(EXTCLIMBERSPEED * 0.5);
                topArticulatingClimber.set(ARTCLIMBERSPEED);
                bottomArticulatingClimber.set(ARTCLIMBERSPEED);
            } else if (thirdButtonTime > 2 && thirdButtonTime <= 3) {
                // Extending arms pull articulating arms up onto bar 2
                topExtendingClimber.set(-EXTCLIMBERSPEED);
                bottomExtendingClimber.set(-EXTCLIMBERSPEED);
                topArticulatingClimber.set(0);
                bottomArticulatingClimber.set(0);
            } else if (thirdButtonTime > 3 && thirdButtonTime <= 4) {
                // Extending arms extend slightly so that they can rotate off of bar 2
                // Articulating arms rotate to angle towards bar 3
                topExtendingClimber.set(EXTCLIMBERSPEED * 0.5);
                bottomExtendingClimber.set(EXTCLIMBERSPEED * 0.5);
                topArticulatingClimber.set(-ARTCLIMBERSPEED);
                bottomArticulatingClimber.set(-ARTCLIMBERSPEED);
            } else if (thirdButtonTime > 4 && thirdButtonTime <= 5) {
                // Extending arms reach to bar 3
                topExtendingClimber.set(EXTCLIMBERSPEED);
                bottomExtendingClimber.set(EXTCLIMBERSPEED);
                topArticulatingClimber.set(0);
                bottomExtendingClimber.set(0);
            } else if (thirdButtonTime > 5 && thirdButtonTime <= 6) {
                // Articulating arms rotate to allow extending arms to grab bar 3
                topExtendingClimber.set(0);
                bottomExtendingClimber.set(0);
                topArticulatingClimber.set(ARTCLIMBERSPEED);
                bottomArticulatingClimber.set(ARTCLIMBERSPEED);
            } else if (thirdButtonTime > 6 && thirdButtonTime <= 7) {
                // Extending arms pull us on to bar 3
                // Articulating arms slowly rotate off of bar 2 to rest against bar 3
                // (so we don't get points deducted for touching bar 2)
                topExtendingClimber.set(-EXTCLIMBERSPEED);
                bottomExtendingClimber.set(-EXTCLIMBERSPEED);
                topArticulatingClimber.set(ARTCLIMBERSPEED * 0.5);
                bottomArticulatingClimber.set(ARTCLIMBERSPEED * 0.5);
            }
        } else {
            // manual time! please use manual please it's great
            // Articulating Climber Controls [MANUAL]: operator buttons 5 & 3
            if (articulatingClimberButton) {
                topArticulatingClimber.set(ARTCLIMBERSPEED);
                bottomArticulatingClimber.set(ARTCLIMBERSPEED);
            } else if (articulatingClimberOtherwayButton) {
                topArticulatingClimber.set(-ARTCLIMBERSPEED);
                bottomArticulatingClimber.set(-ARTCLIMBERSPEED);
            } else {
                topArticulatingClimber.set(0);
                bottomArticulatingClimber.set(0);
            }

            // Extending Climber Winch Controls [MANUAL]: operator buttons 6 & 4
            if (windClimberButton) {
                topExtendingClimber.set(EXTCLIMBERSPEED);
                bottomExtendingClimber.set(EXTCLIMBERSPEED);
            } else if (unwindClimberButton) {
                topExtendingClimber.set(-EXTCLIMBERSPEED);
                bottomExtendingClimber.set(-EXTCLIMBERSPEED);
            } else {
                topExtendingClimber.set(0);
                bottomExtendingClimber.set(0);
            }

        }

    }

    public void limelightSteeringAlign(boolean limelightAlignButtonL, boolean limelightAlignButtonR,
            double limelightTX) {
        limelightLeftSteer = 0.0;
        limelightRightSteer = 0.0;

        // Limelight Align
        if (!limelightAlignButtonL || !limelightAlignButtonR) {
            return;
        }
        double limelightHeadingError = -limelightTX;
        double limelightAlignmentAdjust = 0.0;

        if (limelightTX > 1.0) {
            limelightAlignmentAdjust = (limelightKP * limelightHeadingError) - limelightMinCommand;
        } else if (limelightTX < 1.0) {
            limelightAlignmentAdjust = (limelightKP * limelightHeadingError) + limelightMinCommand;
        }

        limelightLeftSteer += limelightAlignmentAdjust;
        limelightRightSteer -= limelightAlignmentAdjust;

        // Testing Only
        SmartDashboard.putNumber("LimelightLeftSteer", limelightLeftSteer);
        SmartDashboard.putNumber("LimelightRightSteer", limelightRightSteer);
    }

    // TODO: set up method for adjust hood according to distance
    public double calculateLimelightDistance(double limelightTY) {
        final double limelightMountingHeight = 0.0; // inches
        final double limelightMountingAngle = 23; // degrees rotated back from vertical apparently
        final double limelightTargetHeight = 8; // inches
        final double limelightToBumper = 12; // horizontal distance from limelight's camera to outside of bumper
        double limelightDistance = 0.0;

        double limelightTotalAngle = Math.toRadians(limelightMountingAngle + limelightTY);

        // Limelight distance from target in inches
        limelightDistance = (limelightTargetHeight - limelightMountingHeight) / Math.atan(limelightTotalAngle)
                - limelightToBumper;
        return limelightDistance;
    }

    public double calculateLimelightShootingSpeed(double limelightDistance) {
        if (limelightDistance <= CLOSEDISTANCE) {
            limelightShootingRPM = (2870 * Math.pow(1.002, limelightDistance));
        } else if (limelightDistance <= MIDDISTANCE) {
            limelightRightSteer = (2713 * Math.pow(1.002, limelightDistance));
        } else if (limelightDistance <= FARDISTANCE) {
            limelightShootingRPM = (3577 * Math.pow(1.001, limelightDistance));
        } else {
            limelightShootingRPM = 0;
        }
        return limelightShootingRPM;
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

}