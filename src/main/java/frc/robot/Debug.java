package frc.robot;

public class Debug {

    static int level = 4;


    public static void log(int lv, String loglineString){
        if(lv <= level){
            System.out.println(loglineString);
        }
    }
}