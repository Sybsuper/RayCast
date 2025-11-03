package ru.skolkovolab.raycast.entity.tool;

import org.apache.commons.geometry.euclidean.threed.Vector3D;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import ru.skolkovolab.raycast.RayCastTool;
import ru.skolkovolab.raycast.entity.HitBox;
import ru.skolkovolab.raycast.entity.HitBoxGroup;
import ru.skolkovolab.raycast.shared.RayCastIterator;
import ru.skolkovolab.raycast.shared.VecRel;
import ru.skolkovolab.raycast.shared.collision.HitBoxCollision;

import java.util.*;

/**
 * Нужно будет ещё поверх этого итератор написать, не итерировать по каждой группе отдельно.
 * В прицнипе идея простая - какой по расстоянию ближе тот и проверяем первый, дальние не трогаем, пока нет потенциальной возможности пересечения.
 * Не рейкастит назад (ожидаемо)
 *
 * @author sidey383
 **/
public class RayCastHitBoxGroup<T extends HitBox> implements Iterable<HitBoxCollision<T>>, Comparable<RayCastHitBoxGroup<?>> {

    private final double inlet;

    private static final Vector3D[] baseVectorsOne = new Vector3D[]{
            Vector3D.of(1, 0, 0),
            Vector3D.of(0, 1, 0),
            Vector3D.of(0, 0, 1)
    };

    private static final Vector3D[] baseVectorsHalf = new Vector3D[]{
            Vector3D.of(0.5, 0, 0),
            Vector3D.of(0, 0.5, 0),
            Vector3D.of(0, 0, 0.5)
    };

    private final double outlet;

    private final HitBoxGroup<T> group;

    private final Vector3D pos;
    private final Vector3D dir;

    private List<HitBoxCollision<T>> collisions;

    private RayCastHitBoxGroup(HitBoxGroup<T> group, double inlet, double outlet, Vector3D pos, Vector3D dir) {
        this.inlet = inlet;
        this.outlet = outlet;
        this.group = group;
        this.pos = pos;
        this.dir = dir;
    }

    /**
     * @param group group of hitbox for iteration
     * @param pos   start position of raycast
     * @param dir   normalized direction vector
     * @return An empty Optional if the group does not intersect the ray, otherwise a new raycast object
     **/
    public static <T extends HitBox> Optional<RayCastHitBoxGroup<T>> createGroup(HitBoxGroup<T> group, Vector3D pos, Vector3D dir) {
        dir = dir.normalize();
        double dX = pos.getX() - group.getHitBoxGroupCenter().getX();
        double dY = pos.getY() - group.getHitBoxGroupCenter().getY();
        double dZ = pos.getZ() - group.getHitBoxGroupCenter().getZ();
        double c = (dX * dX + dY * dY + dZ * dZ - group.getHitBoxGroupRadius() * group.getHitBoxGroupRadius());
        double halfB = (dX * dir.getX() + dY * dir.getY() + dZ * dir.getZ());
        double D = halfB * halfB - c;
        if (D < 0) {
            return Optional.empty();
        } else {
            double DSqrt = Math.sqrt(D);
            if (DSqrt < halfB)
                return Optional.empty();
            return Optional.of(new RayCastHitBoxGroup<>(group, -halfB - DSqrt, -halfB + DSqrt, pos, dir));
        }
    }

    @Override
    public int compareTo(@NotNull RayCastHitBoxGroup<?> o) {
        return Double.compare(this.inlet, o.inlet);
    }

    @Nullable
    private static <TR extends HitBox> Collision<TR>[] getCollisions(HitBoxGroup<TR> parent,
                                                                     TR hitbox, Vector3D p, Vector3D dir) {
        return switch (hitbox.getHitBoxType()) {
            case BLOCK_DISPLAY -> getBlockCollisions(parent, hitbox, p, dir);
            case ITEM_DISPLAY_NONE -> getItemNoneCollisions(parent, hitbox, p, dir);
            case AABB -> getAABBCollisions(parent, hitbox, p, dir);
        };
    }

    private static <TR extends HitBox> Collision<TR>[] getBlockCollisions(HitBoxGroup<TR> parent,
                                                                          TR hitbox, Vector3D p, Vector3D dir) {
        Vector3D[] directions = new Vector3D[]{baseVectorsOne[0], baseVectorsOne[1], baseVectorsOne[2]};
        RayCastHitBoxUtils.applyRotation(directions, hitbox.getHitBoxRightRotation());
        RayCastHitBoxUtils.applyScale(directions, hitbox.getHitBoxScale());
        RayCastHitBoxUtils.applyRotation(directions, hitbox.getHitBoxLeftRotation());
        double[] intersections = RayCastHitBoxUtils.rhombohedronDownIntersection(p, dir, directions, RayCastTool.minestomToEuclidean(hitbox.getHitBoxPosition()));
        return toCollisions(parent, p, dir, hitbox, intersections);
    }

    private static <TR extends HitBox> Collision<TR>[] getItemNoneCollisions(HitBoxGroup<TR> parent,
                                                                             TR hitbox, Vector3D p, Vector3D dir) {
        Vector3D[] directions = new Vector3D[]{baseVectorsHalf[0], baseVectorsHalf[1], baseVectorsHalf[2]};
        RayCastHitBoxUtils.applyRotation(directions, hitbox.getHitBoxRightRotation());
        RayCastHitBoxUtils.applyScale(directions, hitbox.getHitBoxScale());
        RayCastHitBoxUtils.applyRotation(directions, hitbox.getHitBoxLeftRotation());
        double[] intersections = RayCastHitBoxUtils.rhombohedronCenterIntersection(p, dir, directions, RayCastTool.minestomToEuclidean(hitbox.getHitBoxPosition()));
        return toCollisions(parent, p, dir, hitbox, intersections);
    }

    private static <TR extends HitBox> Collision<TR>[] getAABBCollisions(HitBoxGroup<TR> parent,
                                                                         TR hitbox, Vector3D p, Vector3D dir) {
        // Robust Axis-Aligned Bounding Box ray intersection (slab method)
        var minVec = hitbox.getAABBMin();
        var maxVec = hitbox.getAABBMax();
        if (minVec == null || maxVec == null)
            return null;

        double minX = Math.min(minVec.x(), maxVec.x());
        double minY = Math.min(minVec.y(), maxVec.y());
        double minZ = Math.min(minVec.z(), maxVec.z());
        double maxX = Math.max(minVec.x(), maxVec.x());
        double maxY = Math.max(minVec.y(), maxVec.y());
        double maxZ = Math.max(minVec.z(), maxVec.z());

        double tmin = Double.NEGATIVE_INFINITY;
        double tmax = Double.POSITIVE_INFINITY;

        // X slab
        if (dir.getX() == 0) {
            if (p.getX() < minX || p.getX() > maxX) return null;
        } else {
            double tx1 = (minX - p.getX()) / dir.getX();
            double tx2 = (maxX - p.getX()) / dir.getX();
            if (tx1 > tx2) { double tmp = tx1; tx1 = tx2; tx2 = tmp; }
            if (tx1 > tmin) tmin = tx1;
            if (tx2 < tmax) tmax = tx2;
            if (tmin > tmax) return null;
        }

        // Y slab
        if (dir.getY() == 0) {
            if (p.getY() < minY || p.getY() > maxY) return null;
        } else {
            double ty1 = (minY - p.getY()) / dir.getY();
            double ty2 = (maxY - p.getY()) / dir.getY();
            if (ty1 > ty2) { double tmp = ty1; ty1 = ty2; ty2 = tmp; }
            if (ty1 > tmin) tmin = ty1;
            if (ty2 < tmax) tmax = ty2;
            if (tmin > tmax) return null;
        }

        // Z slab
        if (dir.getZ() == 0) {
            if (p.getZ() < minZ || p.getZ() > maxZ) return null;
        } else {
            double tz1 = (minZ - p.getZ()) / dir.getZ();
            double tz2 = (maxZ - p.getZ()) / dir.getZ();
            if (tz1 > tz2) { double tmp = tz1; tz1 = tz2; tz2 = tmp; }
            if (tz1 > tmin) tmin = tz1;
            if (tz2 < tmax) tmax = tz2;
            if (tmin > tmax) return null;
        }

        double[] intersections = new double[]{tmin, tmax};
        return toCollisions(parent, p, dir, hitbox, intersections);
    }

    @SuppressWarnings("unchecked")
    private static <TR extends HitBox> Collision<TR>[] toCollisions(HitBoxGroup<TR> parent,
                                                                    Vector3D origin, Vector3D direction,
                                                                    TR hitbox, double[] intersections) {
        if (intersections == null)
            return null;
        if (intersections[1] < 0)
            return null;

        VecRel in = intersections[0] < 0 ? new VecRel(origin, 0) : new VecRel(origin, direction, intersections[0]);
        VecRel out = new VecRel(origin, direction, intersections[1]);

        return (Collision<TR>[]) new Collision[]{
                new Collision<>(parent, hitbox, in, out, true),
                new Collision<>(parent, hitbox, in, out, false)
        };
    }

    /**
     * Calculation magic
     **/
    private void initializeCollisionsCollection() {
        if (collisions != null)
            return;
        List<Collision<T>> collisions = new ArrayList<>();
        for (T box : group.getHitBoxCollection()) {
            Collision<T>[] colPair = getCollisions(group, box, pos, dir);
            if (colPair != null) {
                for (Collision<T> tCollision : colPair) {
                    RayCastHitBoxUtils.sortedInsert(collisions, tCollision);
                }
            }
        }
        this.collisions = Collections.unmodifiableList(collisions);
    }

    /**
     * Group entry distance, constant
     **/
    public double inlet() {
        return inlet;
    }

    /**
     * Group exit distance, constant
     **/
    public double outlet() {
        return outlet;
    }

    /**
     * The distance to the first crossing hitbox, initiates calculations
     * If there are no intersections return null
     **/
    public Double actualInlet() {
        initializeCollisionsCollection();
        if (collisions.isEmpty())
            return null;
        else
            return collisions.get(0).distance();
    }

    /**
     * The distance to the last crossing hitbox, initiates calculations
     * If there are no intersections return null
     **/
    public Double actualOutlet() {
        initializeCollisionsCollection();
        if (collisions.isEmpty())
            return null;
        else
            return collisions.get(collisions.size() - 1).distance();
    }

    @NotNull
    @Override
    public RayCastIterator<HitBoxCollision<T>> iterator() {
        initializeCollisionsCollection();
        return new RayCastIterator<>() {

            int i = -1;

            @Override
            public double distance() {
                if (i < 0)
                    return -1;
                if (i >= collisions.size())
                    return Double.MAX_VALUE;
                return collisions.get(i).distance();
            }

            @Override
            public HitBoxCollision<T> current() {
                if (i < 0 || i >= collisions.size())
                    throw new NoSuchElementException();
                return collisions.get(i);
            }

            @Override
            public boolean hasCurrent() {
                return i >= 0 && i < collisions.size();
            }

            @Override
            public boolean hasNext() {
                return i + 1 < collisions.size();
            }

            @Override
            public HitBoxCollision<T> next() {
                if (i + 1 >= collisions.size())
                    throw new NoSuchElementException();
                return collisions.get(++i);
            }
        };
    }
}
