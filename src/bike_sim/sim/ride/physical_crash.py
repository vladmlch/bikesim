"""Crash classification from solved engine contacts, separate from tire grounding."""
import mujoco
import numpy as np


def _crash_geom_ids(model):
    """Resolve crash-relevant geom ids once per compiled model.

    Keyed by ``id(model)``; the entry holds the model itself so an id cannot be
    recycled by a garbage-collected model while it is still cached. Bounded so
    a long-lived process cannot accumulate dead models.
    """
    cache = getattr(physical_contact_crash, '_geom_id_cache', None)
    if cache is None:
        cache = physical_contact_crash._geom_id_cache = {}
    key = id(model)
    entry = cache.get(key)
    if entry is not None and entry[0] is model:
        return entry[1]
    catch, terrain, rider = set(), set(), set()
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
        if name == 'catch_plane':
            catch.add(i)
        elif name == 'terrain':
            terrain.add(i)
        if name.startswith('geom_rider_'):
            rider.add(i)
    ids = (frozenset(catch), frozenset(terrain), frozenset(rider))
    if len(cache) >= 8:
        cache.clear()
    cache[key] = (model, ids)
    return ids


def physical_contact_crash(model, data, load_threshold_n=1.0):
    """Return an emergency cause without treating the catch plane as road.

    Call after the solve and before refreshing endpoint kinematics. Only crash
    geoms selected in the physical builder can produce a rider-ground contact.
    """
    if not np.isfinite(load_threshold_n) or load_threshold_n < 0:
        raise ValueError('contact threshold must be finite and nonnegative')
    catch_ids, terrain_ids, rider_ids = _crash_geom_ids(model)
    wrench=np.zeros(6)
    for i in range(data.ncon):
        contact=data.contact[i]
        g1,g2=int(contact.geom1),int(contact.geom2)
        catch=g1 in catch_ids or g2 in catch_ids
        rider_ground=not catch and (g1 in terrain_ids or g2 in terrain_ids) \
            and (g1 in rider_ids or g2 in rider_ids)
        if not catch and not rider_ground:
            continue
        mujoco.mj_contactForce(model,data,i,wrench)
        if wrench[0] <= load_threshold_n:
            continue
        if catch:
            return 'catch_plane_contact'
        return 'rider_ground_contact'
    return None
