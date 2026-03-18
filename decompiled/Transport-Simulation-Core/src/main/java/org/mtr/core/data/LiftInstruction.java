/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import org.mtr.core.data.LiftDirection;
import org.mtr.core.generated.data.LiftInstructionSchema;
import org.mtr.core.serializer.ReaderBase;

public class LiftInstruction
extends LiftInstructionSchema {
    public LiftInstruction(int floor, LiftDirection direction) {
        super(floor, direction);
    }

    public LiftInstruction(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public int getFloor() {
        return (int)this.floor;
    }

    public LiftDirection getDirection() {
        return this.direction;
    }

    public boolean equals(Object obj) {
        if (obj instanceof LiftInstruction) {
            return this.floor == ((LiftInstruction)obj).floor && this.direction == ((LiftInstruction)obj).direction;
        }
        return super.equals(obj);
    }
}

