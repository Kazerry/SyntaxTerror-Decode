package org.firstinspires.ftc.teamcode.AutoModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueGoalAuto", group = "Autonomous")
public class AutoGoalBlue extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int actionState;

    private DcMotorEx IntakeMotor;
    private DcMotorEx TurretMotor;
    private DcMotorEx OTMotor;

    private CRServo tServo;

    private final Pose startPose = new Pose(22, 125, Math.toRadians(144));
    private final Pose oPose1 = new Pose(44,84);
    private final Pose intakePose1 = new Pose(15,84);
    private final Pose shootPose1 = new Pose(55,84);
    private final Pose intakePose2 = new Pose(15,60);
    private final Pose shootPose2 = new Pose(70,20);

    private Path intake1, shoot1, intake2,oMove1;

    public void buildPaths() {
        oMove1 = new Path(new BezierLine(startPose, oPose1));
        oMove1.setLinearHeadingInterpolation(Math.toRadians(144),Math.toRadians(90));

        intake1 = new Path(new BezierLine(oPose1, intakePose1));
        intake1.setConstantHeadingInterpolation(Math.toRadians(90));

        shoot1 = new Path(new BezierLine(intakePose1, shootPose1));
        shoot1.setConstantHeadingInterpolation(Math.toRadians(90));

        intake2 = new Path(new BezierCurve(shootPose1, new Pose(57, 57), intakePose2));
        intake2.setConstantHeadingInterpolation(Math.toRadians(90));
    }
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                setActionState(0);
                follower.followPath(oMove1);
                setPathState(1);
                break;
            case 1:
                if(follower.atParametricEnd()) {
                    setActionState(1);
                    follower.setMaxPower(0.75);
                    follower.followPath(intake1);
                    setPathState(2);
                }
                break;
            case 2:
                if(follower.atParametricEnd()) {
                    setActionState(0);
                    follower.setMaxPower(1);
                    follower.followPath(shoot1);
                }
                break;
            case 3:
                if(follower.atParametricEnd()) {
                    setActionState(2);
                    if (actionTimer.getElapsedTime() >= 5000) {
                        follower.followPath(intake2);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(follower.atParametricEnd()){

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
                actionTimer.resetTimer();
                OTMotor.setPower(0);
                tServo.setPower(0);
                IntakeMotor.setPower(0);
                //turn things off
                break;
            case 1:
                actionTimer.resetTimer();
                tServo.setPower(10);
                IntakeMotor.setPower(10);
                //Intake stuff
                break;
            case 2:
                actionTimer.resetTimer();
                tServo.setPower(10);
                OTMotor.setPower(10);
                //Outtake stuff!
                break;
            case 3:
                actionTimer.resetTimer();
                break;
            case 4:
                actionTimer.resetTimer();
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
        IntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        OTMotor = hardwareMap.get(DcMotorEx .class, "OTMotor");
        OTMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        OTMotor.setDirection(DcMotorEx.Direction.FORWARD);
        OTMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        tServo = hardwareMap.get(CRServo.class, "tServo");

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

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        if (pathState == 0 && pathTimer.getElapsedTime() >= 1.3){
            follower.setMaxPower(0.3);
        }
    }
}
