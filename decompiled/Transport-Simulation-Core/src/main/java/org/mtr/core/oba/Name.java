/*
 * Decompiled with CFR 0.152.
 */
package org.mtr.core.oba;

import org.mtr.core.generated.oba.NameSchema;
import org.mtr.core.serializer.ReaderBase;

public final class Name
extends NameSchema {
    public Name(ReaderBase readerBase) {
        super(readerBase);
        this.updateData(readerBase);
    }
}

