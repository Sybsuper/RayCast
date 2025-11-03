package ru.skolkovolab.raycast.entity;

import net.minestom.server.coordinate.Vec;
import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation;
import org.jetbrains.annotations.Nullable;

/**
 * @author sidey383
 **/
public interface HitBox {

    enum HitBoxType {
        BLOCK_DISPLAY, ITEM_DISPLAY_NONE, AABB
    }

    Vec getHitBoxPosition();

    QuaternionRotation getHitBoxLeftRotation();

    Vec getHitBoxScale();

    QuaternionRotation getHitBoxRightRotation();

    default HitBoxType getHitBoxType() {
        return HitBoxType.ITEM_DISPLAY_NONE;
    }

    // For AABB hitboxes (axis-aligned), provide min and max corners in world coordinates.
    // Return nulls for non-AABB types.
    default @Nullable Vec getAABBMin() { return null; }
    default @Nullable Vec getAABBMax() { return null; }
}
