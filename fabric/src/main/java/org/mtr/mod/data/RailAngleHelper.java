package org.mtr.mod.data;

import org.mtr.core.data.Position;
import org.mtr.core.tool.Angle;
import org.mtr.libraries.it.unimi.dsi.fastutil.objects.ObjectObjectImmutablePair;

public final class RailAngleHelper {

	private RailAngleHelper() {
	}

	public static ObjectObjectImmutablePair<Angle, Angle> getAngles(Position positionStart, float angleStart, Position positionEnd, float angleEnd) {
		final float railDirection = (float) Math.toDegrees(Math.atan2(positionEnd.getZ() - positionStart.getZ(), positionEnd.getX() - positionStart.getX()));
		final boolean startFacingForward = Math.abs(circularDifference(railDirection, angleStart)) <= 90;
		final boolean endFacingForward = Math.abs(circularDifference(railDirection, angleEnd)) <= 90;
		return new ObjectObjectImmutablePair<>(
				Angle.fromAngle(angleStart + (startFacingForward ? 0 : 180)),
				Angle.fromAngle(angleEnd + (endFacingForward ? 180 : 0))
		);
	}

	private static float circularDifference(float angle1, float angle2) {
		float diff = (angle1 - angle2) % 360;
		if (diff > 180) {
			diff -= 360;
		} else if (diff <= -180) {
			diff += 360;
		}
		return diff;
	}
}
