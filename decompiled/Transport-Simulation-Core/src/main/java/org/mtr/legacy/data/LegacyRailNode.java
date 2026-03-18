/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.legacy.data;

import java.util.function.Consumer;
import org.mtr.core.data.Position;
import org.mtr.core.serializer.ReaderBase;
import org.mtr.core.tool.Utilities;
import org.mtr.legacy.data.DataFixer;
import org.mtr.legacy.data.RailNodeConnection;
import org.mtr.legacy.generated.data.RailNodeSchema;

public final class LegacyRailNode
extends RailNodeSchema {
    public LegacyRailNode(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }

    @Override
    public String getHexId() {
        return Utilities.numberToPaddedHexString(this.node_pos);
    }

    @Override
    public boolean isValid() {
        return true;
    }

    public Position getStartPosition() {
        return DataFixer.fromLong(this.node_pos);
    }

    public long getStartPositionLong() {
        return this.node_pos;
    }

    public void iterateConnections(Consumer<RailNodeConnection> consumer) {
        this.rail_connections.forEach(consumer);
    }
}

