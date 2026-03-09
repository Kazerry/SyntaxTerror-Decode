package org.firstinspires.ftc.teamcode.AutoModes;

import static org.firstinspires.ftc.teamcode.Config.RobotConstants.autoPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config.Localization.LimelightFiducial;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedGoalAuto", group = "Autonomous")
public class AutoGoalRed extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int actionState;

    private DcMotorEx IntakeMotor;
    private DcMotorEx TurretMotor;
    private DcMotorEx OTMotor;
    private CRServo tServo;

    private Limelight3A limelight;
    private LimelightFiducial LimeInit;

    private final Pose startPose = new Pose(122, 125, Math.toRadians(216));
    private final Pose oPose1 = new Pose(100,90);
    private final Pose intakePose1 = new Pose(129,84);
    private final Pose shootPose1 = new Pose(89,84);
    private final Pose intakePose2 = new Pose(129,60);
    private final Pose shootPose2 = new Pose(65,70);
    private final Pose intakePose3 = new Pose(15,35);
    private final Pose shootPose3 = new Pose(67,67);
    private final Pose parkPose = new Pose(40.5,33.5);

    private Path intake1,shoot1,intake2,oMove1,shoot2,intake3,shoot3,park,pMove1,down1;

    public void buildPaths() {
        oMove1 = new Path(new BezierLine(startPose, oPose1));
        oMove1.setLinearHeadingInterpolation(Math.toRadians(216),Math.toRadians(270));

        pMove1 = new Path(new BezierLine(oPose1, new Pose(oPose1.getX(),oPose1.getY()-6)));
        pMove1.setConstantHeadingInterpolation(Math.toRadians(270));

        intake1 = new Path(new BezierLine(oPose1, intakePose1));
        intake1.setConstantHeadingInterpolation(Math.toRadians(270));

        shoot1 = new Path(new BezierLine(intakePose1, shootPose1));
        shoot1.setConstantHeadingInterpolation(Math.toRadians(270));

        intake2 = new Path(new BezierCurve(shootPose1, new Pose(87, 57), intakePose2));
        intake2.setConstantHeadingInterpolation(Math.toRadians(270));

        shoot2 = new Path(new BezierLine(intakePose2, shootPose2));
        shoot2.setConstantHeadingInterpolation(Math.toRadians(270));

        intake3 = new Path(new BezierCurve(shootPose2, new Pose(67, 35), intakePose3));
        intake3.setConstantHeadingInterpolation(Math.toRadians(270));

        shoot3 = new Path(new BezierLine(intakePose3, shootPose3));
        shoot3.setConstantHeadingInterpolation(Math.toRadians(270));

        park = new Path(new BezierLine(shootPose3, parkPose));
        park.setConstantHeadingInterpolation(Math.toRadians(270));

        down1 = new Path(new BezierLine(shootPose1, new Pose(shootPose1.getX(),shootPose1.getY()-20)));
        down1.setConstantHeadingInterpolation(Math.toRadians(270));
    }
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(oMove1);
                OTMotor.setPower(10);
                setPathState(100);
                break;
            case 100:
                if (follower.atParametricEnd()){
                    setActionState(3);
                    setPathState(1);
            }
                break;
            case 1:
                if(actionTimer.getElapsedTimeSeconds() >= 2) {
                    setActionState(1);
                    OTMotor.setPower(-0.1);
                    follower.setMaxPower(0.5);
                    follower.followPath(intake1);
                    setPathState(2);
                }
                break;
            case 2:
                if(follower.atParametricEnd()) {
                    setActionState(0);
                    OTMotor.setPower(10);
                    follower.setMaxPower(1);
                    follower.followPath(shoot1);
                    setPathState(3);
                }
                break;
            case 3:
                if(follower.atParametricEnd()) {
                    setActionState(3);
                    setPathState(4);
                }
                break;
            case 4:
                //extra time for shooting
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(down1);
                    setActionState(0);
                    //follower.followPath(intake2);
                    //follower.setMaxPower(0.5);
                    //setActionState(1);
                    //setPathState(5);
                }
                break;
            case 5:
                if(follower.atParametricEnd()) {
                    follower.setMaxPower(1);
                    setActionState(0);
                    follower.followPath(shoot2);
                    setPathState(6);
                }
                break;
            case 6:
                if(follower.atParametricEnd()) {
                    setActionState(2);
                    setPathState(7);
                }
                break;
            case 7:
                //extra time for shooting
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    setActionState(1);
                    follower.setMaxPower(0.5);
                    follower.followPath(intake3);
                    setPathState(8);
                }
                break;
            case 8:
                if(follower.atParametricEnd()) {
                    follower.setMaxPower(1);
                    setActionState(0);
                    follower.followPath(shoot3);
                    setPathState(9);
                }
                break;
            case 9:
                if(follower.atParametricEnd()) {
                    setActionState(2);
                    setPathState(10);
                }
                break;
            case 10:
                if(actionTimer.getElapsedTimeSeconds() > 2) {
                    setActionState(0);
                    //follower.followPath(park);
                    setPathState(11);
                }
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousActionUpdate(){
        switch (actionState){
            case 0:
                OTMotor.setPower(0);
                tServo.setPower(0);
                IntakeMotor.setPower(0);
                //turn things off
                break;
            case 1:
                tServo.setPower(10);
                IntakeMotor.setPower(10);
                //Intake stuff
                break;
            case 2:
                tServo.setPower(10);
                OTMotor.setPower(10);
                //Outtake stuff!
                break;
            case 3:
                OTMotor.setPower(10);
                tServo.setPower(10);
                IntakeMotor.setPower(10);
                //Everything on
                break;
            case 4:
                break;
        }
    }
    public void setActionState(int aState){
        actionState = aState;
        actionTimer.resetTimer();
    }
    @Override
    public void init() {
        TurretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        TurretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TurretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        TurretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        IntakeMotor = hardwareMap.get(DcMotorEx .class, "IntakeMotor");
        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        OTMotor = hardwareMap.get(DcMotorEx .class, "OTMotor");
        OTMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        OTMotor.setDirection(DcMotorEx.Direction.FORWARD);
        OTMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        tServo = hardwareMap.get(CRServo.class, "tServo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //1 blue 0 red
        LimeInit = new LimelightFiducial();
        LimeInit.LimelightInit(limelight);

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void stop() {
        autoPose = follower.getPose();
    }

    @Override
    public void loop(){
        follower.update();
        double turretPower = LimeInit.getLimePose();
        TurretMotor.setPower(turretPower);
        autonomousPathUpdate();
        autonomousActionUpdate();
        telemetry.addData("actionTimer",actionTimer.getElapsedTimeSeconds());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
    }
}
