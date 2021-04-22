package com.lcrobotics.easyftclib.pathfinding.actions;

public abstract class TriggeredAction {

    private boolean alreadyPerformed = false;

    /**
     * Called regularly by the path it is apart of. If the trigger condition is met this will call doAction().
     */
    public void loop() {
        if (isTriggered()) {
            doAction(alreadyPerformed);
            alreadyPerformed = true;
        }
    }

    /**
     * Resets this action
     */
    public void reset() {
        alreadyPerformed = false;
    }

    /**
     * @return true if the trigger condition is met and the action should be performed, false otherwise.
     */
    public abstract boolean isTriggered();

    /**
     * Perform the triggered action. Automatically called when the trigger condition is met.
     *
     * @param alreadyPerformed True if the action has already been performed, false otherwise.
     */
    public abstract void doAction(boolean alreadyPerformed);
}
