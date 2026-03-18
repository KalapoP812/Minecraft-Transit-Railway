/*
 * Decompiled with CFR 0.152.
 * 
 * Could not load the following classes:
 *  org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList
 */
package org.mtr.core.data;

import org.mtr.core.tool.Utilities;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectArrayList;

public class VehiclePosition {
    private final ObjectArrayList<BlockedSegment> blockedSegments = new ObjectArrayList();

    public void addSegment(double startDistance, double endDistance, long id) {
        this.blockedSegments.add(new BlockedSegment(startDistance, endDistance, id));
    }

    public double getClosestOverlap(double startDistance, double endDistance, boolean reversePositions, long id) {
        double closestOverlap = Double.MAX_VALUE;
        boolean valueSet = false;
        for (BlockedSegment blockedSegment : this.blockedSegments) {
            if (id == blockedSegment.id || !Utilities.isIntersecting(startDistance, endDistance, blockedSegment.startDistance, blockedSegment.endDistance)) continue;
            closestOverlap = reversePositions ? Math.min(closestOverlap, Math.max(0.0, endDistance - blockedSegment.endDistance)) : Math.min(closestOverlap, Math.max(0.0, blockedSegment.startDistance - startDistance));
            valueSet = true;
        }
        return valueSet ? closestOverlap : -1.0;
    }

    private static class BlockedSegment {
        private final double startDistance;
        private final double endDistance;
        private final long id;

        private BlockedSegment(double startDistance, double endDistance, long id) {
            this.startDistance = startDistance;
            this.endDistance = endDistance;
            this.id = id;
        }
    }
}

