package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimeLight {
    Limelight3A limelight;

    public LimeLight(HardwareMap hardwaremap){
        limelight= hardwaremap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(7);
    }

    public int getPatternFromLimelight(){
        LLResult result= limelight.getLatestResult();
        int[] patternList= new int[]{21,22,23};//three patterns
        if(result!=null && result.isValid()){
            List<LLResultTypes.FiducialResult> tagresults =result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tagresults) {
                int id = tag.getFiducialId();
                for (int i = 0; i < patternList.length; i++) {
                    if (id == patternList[i]) {
                        return i;
                        // 21 -> pattern 0
                        // 22 -> pattern 1
                        // 23 -> pattern 2
                    }
                }
            }
        }
        return 0; //By defalt, return pattern #21, avoiding incorrect value that would ruin the program
    }
}