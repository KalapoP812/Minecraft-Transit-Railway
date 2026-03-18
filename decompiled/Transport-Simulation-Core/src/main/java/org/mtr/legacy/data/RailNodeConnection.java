/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.legacy.data;

import org.mtr.core.data.Position;
import org.mtr.core.data.TransportMode;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Angle;
import org.mtr.core.tool.EnumHelper;
import org.mtr.core.tool.Utilities;
import org.mtr.core.tool.Vector;
import org.mtr.legacy.data.DataFixer;
import org.mtr.legacy.generated.data.RailNodeConnectionSchema;

public final class RailNodeConnection
extends RailNodeConnectionSchema {
    public RailNodeConnection(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public Position getEndPosition() {
        return DataFixer.fromLong(this.node_pos);
    }

    public long getEndPositionLong() {
        return this.node_pos;
    }

    public Angle getStartAngle() {
        return this.getAngle(false);
    }

    public Angle getEndAngle() {
        return this.getAngle(true);
    }

    public DataFixer.RailType getRailType() {
        return EnumHelper.valueOf(DataFixer.RailType.WOODEN, this.rail_type);
    }

    public TransportMode getTransportMode() {
        return this.transportMode;
    }

    public String getModelKey() {
        return this.model_key;
    }

    public boolean getIsSecondaryDirection() {
        return this.is_secondary_dir;
    }

    public double getVerticalRadius() {
        return this.vertical_curve_radius;
    }

    private Angle getAngle(boolean reverse) {
        Vector vector1 = this.getPosition(0.0, reverse);
        Vector vector2 = this.getPosition(0.1, reverse);
        return Angle.fromAngle((float)Math.toDegrees(Math.atan2(vector2.z() - vector1.z(), vector2.x() - vector1.x())));
    }

    private Vector getPosition(double rawValue, boolean reverse) {
        double value;
        double count1 = Math.abs(this.t_end_1 - this.t_start_1);
        double count2 = Math.abs(this.t_end_2 - this.t_start_2);
        double clampedValue = Utilities.clamp(rawValue, 0.0, count1 + count2);
        double d = value = reverse ? count1 + count2 - clampedValue : clampedValue;
        if (value <= count1) {
            return RailNodeConnection.getPositionXZ(this.h_1, this.k_1, this.r_1, (double)(this.reverse_t_1 ? -1 : 1) * value + this.t_start_1, this.is_straight_1);
        }
        return RailNodeConnection.getPositionXZ(this.h_2, this.k_2, this.r_2, (double)(this.reverse_t_2 ? -1 : 1) * (value - count1) + this.t_start_2, this.is_straight_2);
    }

    private static Vector getPositionXZ(double h, double k, double r, double t, boolean isStraight) {
        if (isStraight) {
            return new Vector(h * t + k * (Math.abs(h) >= 0.5 && Math.abs(k) >= 0.5 ? 0.0 : r) + 0.5, 0.0, k * t + h * r + 0.5);
        }
        return new Vector(h + r * Math.cos(t / r) + 0.5, 0.0, k + r * Math.sin(t / r) + 0.5);
    }
}

