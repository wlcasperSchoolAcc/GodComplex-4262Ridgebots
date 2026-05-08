package org.firstinspires.ftc.teamcode;

public class MoveScum {
    private double maxV;
    private double maxA;

    public MoveScum(double maxV, double maxA){
        this.maxV = maxV;
        this.maxA = maxA;

    }

    public static class Profile{
        public double pos;
        public double v;
    }
    public Profile calculate(double totDist, double currTime){
        Profile state = new Profile();

        double at = maxV/maxA;
        double ad = 0.5*maxA*at*at;
        double maxVactual = maxV;

        if (ad*2 > totDist){
            ad = totDist/2.0;
            at = Math.sqrt((2*ad)/maxA);
            maxVactual = maxA*at;
        }
        double cruised = totDist- (2*ad);
        double cruiset = (maxVactual == 0) ? 0 : (cruised / maxVactual);

        double totime = (2 * at) + cruiset;

        if(currTime >= totime){
            state.pos = totDist;
            state.v = 0;
            return state;
        }
        if(currTime<at){
            state.pos = 0.5*maxA*(currTime*currTime);
            state.v = maxA * currTime;
        }
     else if (currTime < at + cruiset) {
        double cruiseCurrt = currTime - at;

        state.pos = ad + (maxVactual * cruiseCurrt);
        state.v = maxVactual;

    } else {
        double timeLeft = totime - currTime;
        double distanceLeftToBrake = 0.5 * maxA * (timeLeft * timeLeft);

        state.pos = totDist - distanceLeftToBrake;
        state.v = maxA * timeLeft;
    }
        if (maxA <= 0) throw new IllegalArgumentException("maxA must be positive");

        return state;
    }
}

