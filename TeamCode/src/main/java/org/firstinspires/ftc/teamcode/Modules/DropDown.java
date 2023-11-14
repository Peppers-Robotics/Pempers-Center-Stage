package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IRobotModule;
import org.firstinspires.ftc.teamcode.Robot.IStateBasedModule;
import org.firstinspires.ftc.teamcode.Wrappers.CoolServo;

@Config
public class DropDown implements IStateBasedModule, IRobotModule {

    public static boolean ENABLED = true;

    private final CoolServo servo;
    public static boolean reversedServo = false;

    public static double intakePosition = 0.23, upPosition = 0.9;

    public static double profileMaxVelocity = 20, profileAcceleration = 20;

    public enum State{
        UP(upPosition), GOING_UP(upPosition, UP), INTAKE(intakePosition), GOING_INTAKE(intakePosition, INTAKE);

        public double position;
        public final State nextState;

        State(double position){
            this.position = position;
            this.nextState = this;
        }

        State(double position, State nextState){
            this.position = position;
            this.nextState = nextState;
        }
    }

    private void updateStateValues(){
        State.UP.position = upPosition;
        State.GOING_UP.position = upPosition;
        State.INTAKE.position = intakePosition;
        State.GOING_INTAKE.position = intakePosition;
    }

    private State state;

    private final ElapsedTime timer = new ElapsedTime();

    public State getState(){
        return state;
    }

    public void setState(State newState){
        if(newState == state) return;
        this.state = newState;
        timer.reset();
    }

    public DropDown(Hardware hardware, State initialState){
        if(!ENABLED) servo = null;
        else servo = new CoolServo(hardware.sch3, reversedServo, profileMaxVelocity, profileAcceleration, initialState.position);
        timer.startTime();
        setState(initialState);
    }

    @Override
    public void initUpdate() {
        update();
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updateStateValues();
        updateHardware();
        updateState();
    }

    @Override
    public void updateState() {
        if(servo.getTimeToMotionEnd() == 0) state = state.nextState;
    }

    @Override
    public void updateHardware() {
        servo.setPosition(state.position);

        servo.update();
    }
}
