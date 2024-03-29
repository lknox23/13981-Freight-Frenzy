package org.firstinspires.ftc.teamcode.hardware.Pixy;

import com.qualcomm.robotcore.util.TypeConversion;

import java.util.Vector;

/**
 * Created by CherryPi on 12/29/2017.
 */

public class PixyBlockList extends Vector<PixyBlock> {
    public final int totalCount;

    PixyBlockList(byte totalCount) {
        this.totalCount = TypeConversion.unsignedByteToInt(totalCount);
    }
}