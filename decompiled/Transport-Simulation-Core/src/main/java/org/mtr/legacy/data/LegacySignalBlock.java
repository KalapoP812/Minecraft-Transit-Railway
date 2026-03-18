/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.legacy.data;

import java.util.UUID;
import org.mtr.core.data.Position;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.DataFixer;
import org.mtr.legacy.generated.data.SignalBlockSchema;

public final class LegacySignalBlock
extends SignalBlockSchema {
    public LegacySignalBlock(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    @Override
    public String getHexId() {
        return Utilities.numberToPaddedHexString(this.color);
    }

    @Override
    public boolean isValid() {
        return true;
    }

    public int getColor() {
        return Color.values()[(int)this.color].color;
    }

    public boolean isRail(Position position1, Position position2) {
        return this.rails.stream().anyMatch(railUuid -> {
            UUID uuid = UUID.fromString(railUuid);
            long position1Long = DataFixer.asLong(position1);
            long position2Long = DataFixer.asLong(position2);
            return position1Long == uuid.getLeastSignificantBits() && position2Long == uuid.getMostSignificantBits() || position2Long == uuid.getLeastSignificantBits() && position1Long == uuid.getMostSignificantBits();
        });
    }

    private static enum Color {
        WHITE(0xFFFFFF),
        ORANGE(14188339),
        MAGENTA(11685080),
        LIGHT_BLUE(6724056),
        YELLOW(0xE5E533),
        LIME(8375321),
        PINK(15892389),
        GRAY(0x4C4C4C),
        LIGHT_GRAY(0x999999),
        CYAN(5013401),
        PURPLE(8339378),
        BLUE(3361970),
        BROWN(6704179),
        GREEN(6717235),
        RED(0x993333),
        BLACK(0x191919);

        private final int color;

        private Color(int color) {
            this.color = color;
        }
    }
}

