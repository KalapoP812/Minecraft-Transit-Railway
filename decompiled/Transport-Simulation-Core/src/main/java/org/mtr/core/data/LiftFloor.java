/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.data;

import org.mtr.core.data.Position;
import org.mtr.core.generated.data.LiftFloorSchema;
import org.mtr.core.serializer.ReaderBase;

public class LiftFloor
extends LiftFloorSchema {
    public LiftFloor(Position position) {
        super(position);
    }

    public LiftFloor(Position position, String number, String description) {
        super(position);
        this.setNumberAndDescription(number, description);
    }

    public LiftFloor(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public Position getPosition() {
        return this.position;
    }

    public String getNumber() {
        return this.number;
    }

    public String getDescription() {
        return this.description;
    }

    public void setNumberAndDescription(String number, String description) {
        this.number = number;
        this.description = description;
    }
}

