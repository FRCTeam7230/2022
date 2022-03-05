package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.CANSparkMax;
//contains intake, conveyor, shooter, and climb
// sudokode
public class Mechanism{
    private boolean operating, polarity;
    private String mode;
    private int axis, canID1, canID2, forward, backward;
    private double power; 
    private Joystick m_stick = new Joystick(0);
    private CANSparkMax motor1 = new CANSparkMax(canID1, CANSparkMax.MotorType.kBrushless);
    //abstract
    
    //input to end mechanism (also emergeny stop)
    //mechanism state output
    public Mechanism(){
        this("button",1,4,7,0.5);
        operating=false;
        polarity = false;
    }
    public Mechanism(String mode, int canID1, int forward, int backward, double power)
    {
        this.mode = mode;
        this.canID1 =canID1;
        this.forward = forward;
        this.backward = backward;
        this.power = power;
    }
    public boolean getState(){
        return operating;
    }

    public void flipPolarity(boolean direction){
        polarity = direction;
    }
    //input to starm mechanism
    public void start(Boolean is){
        operating = is;
    }
   
    public void addControlAxis(int a)
    {
        axis = a;
    }
    public void addForwardButton(int b)
    {
        forward =b;
    }
    public void addBackwardButton(int b)
    {
        backward = b;
    }
    public void setMode(String s){
        if(s.equalsIgnoreCase("button")||s.equalsIgnoreCase("axis"))
            mode = s;
    }
    public void setPower(int p){
        power = p;
    }
    public int getCanID1(){
        return canID1;
    }
    public int getCanID2(){
        return canID2;
    }
    public int getAxis(){
        return axis;
    }
    public int getForward(){
        return forward;
    }
    public int getBackward(){
        return backward;
    }
    public double getPower(){
        return power;
    }
    public String getMode(){
        return mode;
    }
    public void run(){
        // DriverStation.reportWarning("running",true);
        if(mode.equalsIgnoreCase("axis")){
            DriverStation.reportWarning("running axis",true);
            motor1.set(m_stick.getRawAxis(axis));
        }
        else if(mode.equalsIgnoreCase("button")){
            DriverStation.reportWarning("running button, CANID=" + canID1,true);
            // if(m_stick.getRawButton(forward)){
            //     DriverStation.reportWarning("running fwd",true);
            //     motor1.set(power);
            // }
            // else if(m_stick.getRawButton(backward)){
            //     DriverStation.reportWarning("running bkwd",true);
            //     motor1.set(-power); 
            // }
            motor1.set(power);
        }


    }
    public void go(double power){
        motor1.set(power);
    }
    public void stop(){
        motor1.set(0);
    }
    
}