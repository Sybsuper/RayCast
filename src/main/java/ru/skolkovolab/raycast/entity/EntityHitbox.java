package ru.skolkovolab.raycast.entity;

import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityType;
import net.minestom.server.entity.LivingEntity;
import net.minestom.server.entity.attribute.Attribute;
import net.minestom.server.entity.metadata.display.AbstractDisplayMeta;
import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class EntityHitbox implements HitBox {
    private final HitBoxType type;
    private final Entity entity;

    public EntityHitbox(Entity entity) {
        this.entity = entity;
        EntityType type = entity.getEntityType();
        if (type.equals(EntityType.ITEM_DISPLAY)) {
            this.type = HitBoxType.ITEM_DISPLAY_NONE;
        } else if (type.equals(EntityType.BLOCK_DISPLAY)) {
            this.type = HitBoxType.BLOCK_DISPLAY;
        } else {
            this.type = HitBoxType.AABB;
        }
    }

    public @NotNull Entity getEntity() {
        return this.entity;
    }

    public @NotNull AbstractDisplayMeta getEntityMeta() {
        return (AbstractDisplayMeta) entity.getEntityMeta();
    }

    @Override
    public Vec getHitBoxPosition() {
        if (this.type == HitBoxType.AABB) return entity.getPosition().asVec();
        return entity.getPosition().add(getEntityMeta().getTranslation()).asVec();
    }

    @Override
    public Vec getHitBoxScale() {
        if (this.type == HitBoxType.AABB) {
            if (this.entity instanceof LivingEntity livingEntity) {
                double scale = livingEntity.getAttributeValue(Attribute.SCALE);
                return new Vec(scale);
            } else {
                return Vec.ONE;
            }
        }
        return getEntityMeta().getScale();
    }

    @Override
    public QuaternionRotation getHitBoxLeftRotation() {
        float[] floats = getEntityMeta().getLeftRotation();
        return QuaternionRotation.of(floats[3], floats[0], floats[1], floats[2]);
    }

    @Override
    public QuaternionRotation getHitBoxRightRotation() {
        float[] floats = getEntityMeta().getRightRotation();
        return QuaternionRotation.of(floats[3], floats[0], floats[1], floats[2]);
    }

    @Override
    public HitBoxType getHitBoxType() {
        return this.type;
    }

    @Override
    public @Nullable Vec getAABBMin() {
        Vec start = entity.getBoundingBox().relativeStart();
        return start.add(entity.getPosition());
    }

    @Override
    public @Nullable Vec getAABBMax() {
        Vec end = entity.getBoundingBox().relativeEnd();
        return end.add(entity.getPosition());
    }
}
