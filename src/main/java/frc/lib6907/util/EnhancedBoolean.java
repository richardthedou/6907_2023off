package frc.lib6907.util;

public class EnhancedBoolean {
    private boolean raw_value;
    private boolean last_raw;
    private boolean FtoT, TToF;
    
    public EnhancedBoolean() {
        this.raw_value = false;
        this.last_raw = false;
        this.FtoT = false;
        this.TToF = false;
    }

    /**
     * update the boolean with new raw value
     */
    public void update(boolean rawValue) {
        raw_value = rawValue;
        if (!last_raw && raw_value) {
            FtoT = true;
        } else if (last_raw && !raw_value) {
            TToF = true;
        }
        last_raw = raw_value;
    }

    public boolean getRaw() {
        return raw_value;
    }

    public boolean getFtoT() {
        if (FtoT) {
            FtoT = false;
            return true;
        }
        return false;
    }
    

    public boolean getTToF() {
        if (TToF) {
            TToF = false;
            return true;
        }
        return false;
    }
}
