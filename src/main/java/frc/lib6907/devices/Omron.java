package frc.lib6907.devices;

import edu.wpi.first.wpilibj.DigitalInput;

public class Omron {
    private boolean state;
    private boolean lastState;
    private boolean reversed;
    private DigitalInput din;

    //手动构造方法
    public Omron(int id){
        this.din = new DigitalInput(id);
        this.lastState = false;
        this.state = false;
        this.reversed = true;
    }

    public void setReversed(boolean rev){
        reversed = rev;
    }

    public boolean update(){
        lastState = state;
        state = din.get();
        if (reversed) state = !state;
        return state;
    }

    public boolean fromFtoT(){
        boolean ret = false;
        if(state == true && lastState == false){ //上次状态低电压，这次状态高电压
            ret = true;
        }
        return ret;
    }

    public boolean fromTtoF(){
        boolean ret = false;
        if(state == false && lastState == true){ //上次状态高电压，这次状态低电压
            ret = true;
        }
        return ret;
        
    }
}
