/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.operation;

import org.mtr.core.data.Lift;
import org.mtr.core.data.LiftDirection;
import org.mtr.core.data.LiftInstruction;
import org.mtr.core.data.Position;
import org.mtr.core.generated.operation.PressLiftInstructionSchema;
import org.mtr.core.serializer.ReaderBase;

public final class PressLiftInstruction
extends PressLiftInstructionSchema {
    PressLiftInstruction(Position position, LiftDirection direction) {
        super(position, direction);
    }

    public PressLiftInstruction(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    LiftInstruction getLiftInstruction(Lift lift) {
        int floor = lift.getFloorIndex(this.position);
        return floor >= 0 ? new LiftInstruction(floor, this.direction) : null;
    }
}

