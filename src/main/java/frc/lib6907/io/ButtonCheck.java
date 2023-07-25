package frc.lib6907.io;

import edu.wpi.first.wpilibj.Timer;

public class ButtonCheck{
    boolean buttonCheck = false;
    boolean buttonActive = false;
    boolean activationReported = false;
    boolean longPressed = false;
    boolean longPressActivated = false;
    boolean hasBeenPressed = false;
    boolean longReleased = false;
    private double buttonStartTime = 0;
    private double longPressDuration = 0.25;
    public void setLongPressDuration(double seconds){
        longPressDuration = seconds;
    }
    
    public ButtonCheck(){

    }
    public void update(boolean buttonCheck){
        if(buttonCheck){
            if(buttonActive){
                if(((Timer.getFPGATimestamp() - buttonStartTime) > longPressDuration) && !longPressActivated){
                    longPressActivated = true;
                    longPressed = true;
                    longReleased = false;
                }
            }else{
                buttonActive = true;
                activationReported = false;
                buttonStartTime = Timer.getFPGATimestamp();
            }
        }else{
            if(buttonActive){
                buttonActive = false;
                activationReported = true;
                if(longPressActivated){
                    hasBeenPressed = false;
                    longPressActivated = false;
                    longPressed = false;
                    longReleased = true;
                }else{
                    hasBeenPressed = true;
                }
            }
        }
    }

    /** Returns true once the button is pressed, regardless of
     *  the activation duration. Only returns true one time per
     *  button press, and is reset upon release.
     */
    public boolean wasActivated(){
        if(buttonActive && !activationReported){
            activationReported = true;
            return true;
        }
        return false;
    }
    
    /** Returns true once the button is released after being
     *  held for 0.25 seconds or less. Only returns true one time
     *  per button press.
     */
    public boolean shortReleased(){
        if(hasBeenPressed){
            hasBeenPressed = false;
            return true;
        }
        return false;
    }
    
    /** Returns true once if the button is pressed for more than 0.25 seconds.
     *  Only true while the button is still depressed; it becomes false once the 
     *  button is released.
     */
    public boolean longPressed(){
        if(longPressed){
            longPressed = false;
            return true;
        }
        return false;
    }
    
    /** Returns true one time once the button is released after being held for
     *  more than 0.25 seconds.
     */
    public boolean longReleased(){
        if(longReleased){
            longReleased = false;
            return true;
        }
        return false;
    }

    /** Returns true once the button is released, regardless of activation duration. */
    public boolean wasReleased(){
        return shortReleased() || longReleased();
    }
    
    /** Returns true if the button is currently being pressed. */
    public boolean isBeingPressed(){
        return buttonActive;
    }
}
