/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.operation;

import org.mtr.core.data.Lift;
import org.mtr.core.data.LiftDirection;
import org.mtr.core.data.LiftInstruction;
import org.mtr.core.data.Position;
import org.mtr.core.generated.operation.PressLiftSchema;
import org.mtr.core.operation.PressLiftInstruction;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.simulation.Simulator;

public final class PressLift
extends PressLiftSchema {
    public PressLift() {
    }

    public PressLift(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    public void add(Position position, LiftDirection direction) {
        this.instructions.add(new PressLiftInstruction(position, direction));
    }

    public void pressLift(Simulator simulator) {
        double lowestDistance = Double.MAX_VALUE;
        Lift selectedLift = null;
        LiftInstruction selectedLiftInstruction = null;
        block0: for (Lift lift : simulator.lifts) {
            for (PressLiftInstruction pressLiftInstruction : this.instructions) {
                double distance;
                LiftInstruction liftInstruction = pressLiftInstruction.getLiftInstruction(lift);
                if (liftInstruction == null || !((distance = lift.pressButton(liftInstruction, false)) < lowestDistance)) continue;
                lowestDistance = distance;
                selectedLift = lift;
                selectedLiftInstruction = liftInstruction;
                continue block0;
            }
        }
        if (selectedLift != null) {
            selectedLift.pressButton(selectedLiftInstruction, true);
        }
    }
}

