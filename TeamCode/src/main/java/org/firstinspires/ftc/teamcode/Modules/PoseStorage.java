package org.firstinspires.ftc.teamcode.Modules;
import android.content.Context;
import android.content.SharedPreferences;
public class PoseStorage {
    public static class PoseStorageData{
        public double x;
        public double y;
        public double heading;
        public boolean alliance;
        public PoseStorageData(double x,double y, double heading, boolean alliance){
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.alliance = alliance;
        }
    }

    private static final String PREF_NAME = "OdometryPose";

    private static final String KEY_X = "x";
    private static final String KEY_Y = "y";
    private static final String KEY_HEADING = "heading";
    private static final String KEY_ALLIANCE = "alliance";

    public static void savePose(Context context, PoseStorageData poseStorageData) {
        SharedPreferences prefs =
                context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);

        prefs.edit()
                .putFloat(KEY_X, (float) poseStorageData.x)
                .putFloat(KEY_Y, (float) poseStorageData.y)
                .putFloat(KEY_HEADING, (float) poseStorageData.heading)
                .putBoolean(KEY_ALLIANCE, poseStorageData.alliance) // red / blue
                .apply();
    }

    public static PoseStorageData loadPose(Context context) {
        SharedPreferences prefs =
                context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);

        double x = prefs.getFloat(KEY_X, 0);
        double y = prefs.getFloat(KEY_Y, 0);
        double heading = prefs.getFloat(KEY_HEADING, 0);
        boolean alliance = prefs.getBoolean(KEY_ALLIANCE, false); // red / blue

        return new PoseStorageData(x,y,heading,alliance);
    }
}
