/**************************************************************************/
/*  jolt_contact_listener_3d.cpp                                          */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "jolt_contact_listener_3d.h"

#include "../jolt_project_settings.h"
#include "../misc/jolt_type_conversions.h"
#include "../objects/jolt_area_3d.h"
#include "../objects/jolt_body_3d.h"
#include "../objects/jolt_soft_body_3d.h"
#include "jolt_space_3d.h"

#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>

void JoltContactListener3D::OnContactAdded(const JPH::Body &p_body1, const JPH::Body &p_body2, const JPH::ContactManifold &p_manifold, JPH::ContactSettings &p_settings) {
	_try_override_collision_response(p_body1, p_body2, p_settings);
	_try_limit_mass_ratio(p_body1, p_body2, p_manifold, p_settings);
	_note_resting_load(p_body1, p_body2, p_manifold, p_settings);
	_try_apply_surface_velocities(p_body1, p_body2, p_settings);
	_try_add_contacts(p_body1, p_body2, p_manifold, p_settings);
	_try_evaluate_area_overlap(p_body1, p_body2, p_manifold.mSubShapeID1, p_manifold.mSubShapeID2);

#ifdef DEBUG_ENABLED
	_try_add_debug_contacts(p_body1, p_body2, p_manifold);
#endif
}

void JoltContactListener3D::OnContactPersisted(const JPH::Body &p_body1, const JPH::Body &p_body2, const JPH::ContactManifold &p_manifold, JPH::ContactSettings &p_settings) {
	_try_override_collision_response(p_body1, p_body2, p_settings);
	_try_limit_mass_ratio(p_body1, p_body2, p_manifold, p_settings);
	_note_resting_load(p_body1, p_body2, p_manifold, p_settings);
	_try_apply_surface_velocities(p_body1, p_body2, p_settings);
	_try_add_contacts(p_body1, p_body2, p_manifold, p_settings);
	_try_evaluate_area_overlap(p_body1, p_body2, p_manifold.mSubShapeID1, p_manifold.mSubShapeID2);

#ifdef DEBUG_ENABLED
	_try_add_debug_contacts(p_body1, p_body2, p_manifold);
#endif
}

void JoltContactListener3D::OnContactRemoved(const JPH::SubShapeIDPair &p_shape_pair) {
	if (_try_remove_contacts(p_shape_pair)) {
		return;
	}

	if (_try_remove_area_overlap(p_shape_pair)) {
		return;
	}
}

JPH::SoftBodyValidateResult JoltContactListener3D::OnSoftBodyContactValidate(const JPH::Body &p_soft_body, const JPH::Body &p_other_body, JPH::SoftBodyContactSettings &p_settings) {
	_try_override_collision_response(p_soft_body, p_other_body, p_settings);
	return JPH::SoftBodyValidateResult::AcceptContact;
}

void JoltContactListener3D::OnSoftBodyContactAdded(const JPH::Body &p_soft_body, const JPH::SoftBodyManifold &p_manifold) {
	for (JPH::uint i = 0; i < p_manifold.GetNumSensorContacts(); i++) {
		if (JPH::Body *other_jolt_body = space->try_get_jolt_body(p_manifold.GetSensorContactBodyID(i))) {
			_try_evaluate_area_overlap(p_soft_body, *other_jolt_body, JPH::SubShapeID(), JPH::SubShapeID());
		}
	}

#ifdef DEBUG_ENABLED
	_try_add_debug_contacts(p_soft_body, p_manifold);
#endif
}

bool JoltContactListener3D::_try_override_collision_response(const JPH::Body &p_jolt_body1, const JPH::Body &p_jolt_body2, JPH::ContactSettings &p_settings) {
	if (p_jolt_body1.IsSensor() || p_jolt_body2.IsSensor()) {
		return false;
	}

	if (!p_jolt_body1.IsDynamic() && !p_jolt_body2.IsDynamic()) {
		return false;
	}

	const JoltBody3D *body1 = reinterpret_cast<JoltBody3D *>(p_jolt_body1.GetUserData());
	const JoltBody3D *body2 = reinterpret_cast<JoltBody3D *>(p_jolt_body2.GetUserData());

	const bool can_collide1 = body1->can_collide_with(*body2);
	const bool can_collide2 = body2->can_collide_with(*body1);

	if (can_collide1 && !can_collide2) {
		p_settings.mInvMassScale2 = 0.0f;
		p_settings.mInvInertiaScale2 = 0.0f;
	} else if (can_collide2 && !can_collide1) {
		p_settings.mInvMassScale1 = 0.0f;
		p_settings.mInvInertiaScale1 = 0.0f;
	}

	return true;
}

bool JoltContactListener3D::_try_override_collision_response(const JPH::Body &p_jolt_soft_body, const JPH::Body &p_jolt_other_body, JPH::SoftBodyContactSettings &p_settings) {
	if (p_jolt_other_body.IsSensor()) {
		return false;
	}

	const JoltSoftBody3D *soft_body = reinterpret_cast<JoltSoftBody3D *>(p_jolt_soft_body.GetUserData());
	const JoltBody3D *other_body = reinterpret_cast<JoltBody3D *>(p_jolt_other_body.GetUserData());

	const bool can_collide1 = soft_body->can_collide_with(*other_body);
	const bool can_collide2 = other_body->can_collide_with(*soft_body);

	if (can_collide1 && !can_collide2) {
		p_settings.mInvMassScale2 = 0.0f;
		p_settings.mInvInertiaScale2 = 0.0f;
	} else if (can_collide2 && !can_collide1) {
		p_settings.mInvMassScale1 = 0.0f;
	}

	return true;
}

// Jolt's iterative solver can't let a light body carry a much heavier one: the
// heavy body sinks it into what it rests on. Within this contact only, when the
// heavier of two dynamic bodies rests on the lighter one (the contact normal
// within 60 degrees of up, the heavy body's centre of mass above), the lighter one acts as heavy
// as the heavier one divided by max_contact_mass_ratio (Jolt's ContactSettings
// inverse mass scale, as in its ModifyMassTest sample). Only from above, so a
// light body thrown or pushed sideways at a heavy one still hits like a light
// one (momentum isn't conserved in a scaled contact). The masses don't change
// between steps, so the scale stays the same for a resting pair over time, as
// Jolt asks. Off at 0. Contacts already made one-way
// (_try_override_collision_response) are left as they are.
bool JoltContactListener3D::_try_limit_mass_ratio(const JPH::Body &p_jolt_body1, const JPH::Body &p_jolt_body2, const JPH::ContactManifold &p_manifold, JPH::ContactSettings &p_settings) {
	const float max_ratio = JoltProjectSettings::max_contact_mass_ratio;
	if (max_ratio <= 0.0f) {
		return false;
	}

	if (!p_jolt_body1.IsDynamic() || !p_jolt_body2.IsDynamic()) {
		return false;
	}

	if (p_settings.mInvMassScale1 != 1.0f || p_settings.mInvMassScale2 != 1.0f) {
		return false;
	}

	const float inverse_mass1 = p_jolt_body1.GetMotionPropertiesUnchecked()->GetInverseMass();
	const float inverse_mass2 = p_jolt_body2.GetMotionPropertiesUnchecked()->GetInverseMass();
	if (inverse_mass1 <= 0.0f || inverse_mass2 <= 0.0f) {
		return false;
	}

	// Godot applies gravity itself (Jolt's is zero): the default area has it.
	const JoltArea3D *default_area = space->get_default_area();
	if (default_area == nullptr) {
		return false;
	}

	const JPH::Vec3 gravity = to_jolt(default_area->get_gravity_vector());
	if (gravity.LengthSq() <= 0.0f) {
		return false;
	}

	const JPH::Vec3 up = -gravity.Normalized();
	constexpr float ON_TOP = 0.5f; // cos(60 degrees)
	if (JPH::abs(p_manifold.mWorldSpaceNormal.Dot(up)) < ON_TOP) {
		return false;
	}

	// Which one is above, by their centres of mass (not the normal's sign).
	const float body2_above = JPH::Vec3(p_jolt_body2.GetCenterOfMassPosition() - p_jolt_body1.GetCenterOfMassPosition()).Dot(up);

	if (inverse_mass1 > inverse_mass2 * max_ratio && body2_above > 0.0f) {
		const float scale = inverse_mass2 * max_ratio / inverse_mass1;
		p_settings.mInvMassScale1 = scale;
		p_settings.mInvInertiaScale1 = scale;
		return true;
	}

	if (inverse_mass2 > inverse_mass1 * max_ratio && body2_above < 0.0f) {
		const float scale = inverse_mass1 * max_ratio / inverse_mass2;
		p_settings.mInvMassScale2 = scale;
		p_settings.mInvInertiaScale2 = scale;
		return true;
	}

	return false;
}

// _lift_masses lifts a body to its load / (ratio x this), clear of the ratio: at
// the ratio exactly, a contact with max_contact_mass_ratio at the same value
// switched its scale on and off with every small change in the load, and the stack
// shook. _note_resting_load skips the bodies on top lighter than true mass x ratio
// x this, which could never lift it.
static constexpr float LIFT_MARGIN = 0.8f;

// Lock-free float add and max, for loads written by several threads.
static void atomic_add(std::atomic<float> &r_value, float p_add) {
	float old = r_value.load(std::memory_order_relaxed);
	while (!r_value.compare_exchange_weak(old, old + p_add, std::memory_order_relaxed)) {
	}
}

static void atomic_max(std::atomic<float> &r_value, float p_other) {
	float old = r_value.load(std::memory_order_relaxed);
	while (p_other > old && !r_value.compare_exchange_weak(old, p_other, std::memory_order_relaxed)) {
	}
}

// Adds the vertical part of the last step's impulse of a resting contact between
// two dynamic bodies (the normal within 60 degrees of up) to the lower body's
// load, for _lift_masses.
void JoltContactListener3D::_note_resting_load(const JPH::Body &p_jolt_body1, const JPH::Body &p_jolt_body2, const JPH::ContactManifold &p_manifold, JPH::ContactSettings &p_settings) {
	if (JoltProjectSettings::max_resting_mass_ratio <= 0.0f || resting_gravity <= 0.0f) {
		return;
	}

	if (!p_jolt_body1.IsDynamic() || !p_jolt_body2.IsDynamic() || p_jolt_body1.IsSensor() || p_jolt_body2.IsSensor()) {
		return;
	}

	const float along_up = p_manifold.mWorldSpaceNormal.Dot(resting_up);
	if (JPH::abs(along_up) < 0.5f) {
		return;
	}

	// The impulse acts along the normal on body 2: body 1 is below when it points up.
	const JPH::Body &lower = along_up > 0.0f ? p_jolt_body1 : p_jolt_body2;
	const JPH::Body &upper = along_up > 0.0f ? p_jolt_body2 : p_jolt_body1;
	JoltBody3D *body = reinterpret_cast<JoltBody3D *>(lower.GetUserData());

	// A body anchored to the world (a claw's or a hand's own body, on its drive)
	// presses with its drive, not its weight: the empty claw pressed into a pile
	// lifted the pill and the cube under it, and the hand shook.
	if (reinterpret_cast<JoltBody3D *>(upper.GetUserData())->is_anchored()) {
		return;
	}

	// A body on top too light to ever lift this one (the load is capped by it, in
	// _lift_masses) counts for nothing: skipping it here keeps a pile of like
	// boxes out of the bookkeeping and the lists (a 300-box pile cost 11% more).
	const float upper_mass_now = 1.0f / upper.GetMotionPropertiesUnchecked()->GetInverseMass();
	if (upper_mass_now <= body->get_mass() * JoltProjectSettings::max_resting_mass_ratio * LIFT_MARGIN) {
		return;
	}

	// A contact with no impulse yet (new, or about to touch: Jolt finds contacts
	// a little ahead) with a much heavier body on top: expect its weight at once,
	// so the light body is heavy before the impact instead of a few steps after
	// (a 60 kg anvil dropped 30 cm sank 67 mm into a can, waiting for the load).
	const float upper_mass = reinterpret_cast<JoltBody3D *>(upper.GetUserData())->get_mass();
	const bool anticipate = p_settings.mPreviousNormalImpulse <= 0.0f && upper_mass > body->get_mass() * JoltProjectSettings::max_resting_mass_ratio;
	if (p_settings.mPreviousNormalImpulse <= 0.0f && !anticipate) {
		return;
	}

	// Until the lift lands (next step), in this contact only, the light body acts as
	// heavy as it will be lifted, so a body landing on it fast doesn't sink in on the
	// step of the impact (an anvil dropped 30 cm dipped 24 mm into a can). Safe to
	// switch: the contact has no impulse for Jolt to start from, and next step's real
	// mass is the same.
	if (anticipate && p_settings.mInvMassScale1 == 1.0f && p_settings.mInvMassScale2 == 1.0f) {
		const float lifted = upper_mass / (JoltProjectSettings::max_resting_mass_ratio * LIFT_MARGIN);
		const float light_now = 1.0f / lower.GetMotionPropertiesUnchecked()->GetInverseMass();
		if (lifted > light_now) {
			const float scale = light_now / lifted;
			if (&lower == &p_jolt_body1) {
				p_settings.mInvMassScale1 = scale;
				p_settings.mInvInertiaScale1 = scale;
			} else {
				p_settings.mInvMassScale2 = scale;
				p_settings.mInvInertiaScale2 = scale;
			}
		}
	}

	// No lock: Jolt calls this from several threads at once, and a lock here (shared
	// with the contact reports) made a pile with an anvil on it 5% slower.
	if (body->resting_listed_step.exchange(lift_step, std::memory_order_relaxed) != lift_step) {
		const uint32_t slot = loaded_count.fetch_add(1, std::memory_order_relaxed);
		if (slot < loaded_bodies.size()) {
			loaded_bodies[slot] = lower.GetID();
		}
	}
	atomic_max(body->resting_load_cap, upper_mass_now);
	if (anticipate) {
		atomic_max(body->anticipated_load, upper_mass);
	} else {
		atomic_add(body->resting_load_impulse, p_settings.mPreviousNormalImpulse * JPH::abs(along_up));
	}
}

// Jolt's iterative solver can't hold a much heavier body up on a light one. With
// max_resting_mass_ratio, a body carrying a load (_note_resting_load, the last
// step's) really gets heavier: at least load / ratio, inertia scaled alike, so
// every contact and joint sees the same masses and passes the whole weight on.
// (Use it instead of max_contact_mass_ratio: both on one pair, the contact scale
// switched as the lift came and went, and a crate with a box in it shook apart.)
// Gravity still uses its true mass (JoltBody3D::_integrate_forces). The load is
// smoothed over about a quarter second (a jerk pushes harder than the weight,
// and lifting for it made a held tray lag twice as far), and the mass changes at
// most x1.25 up or x0.8 down a step. It never adds energy: going up, momentum is
// kept (velocities scale down); going down, velocities are kept. Sleeping
// bodies keep their mass. One pass over the bodies loaded or lifted.
void JoltContactListener3D::_lift_masses() {
	const float ratio = JoltProjectSettings::max_resting_mass_ratio;
	const float step = space->get_last_step();

	// Up and gravity for this step's contacts (_note_resting_load).
	const JoltArea3D *default_area = space->get_default_area();
	const Vector3 gravity = default_area != nullptr ? default_area->get_gravity_vector() : Vector3();
	resting_gravity = gravity.length();
	resting_up = resting_gravity > 0.0f ? to_jolt(-gravity / resting_gravity) : JPH::Vec3::sZero();
	const float g = resting_gravity;

	JPH::PhysicsSystem &system = space->get_physics_system();
	const JPH::BodyLockInterfaceNoLock &bodies = system.GetBodyLockInterfaceNoLock();

	// Every body loaded in the last step (listed once each by its stamp) or still
	// lifted, each once (by a second stamp). The lists keep their capacity.
	const uint32_t loaded = MIN(loaded_count.load(std::memory_order_relaxed), loaded_bodies.size());
	const uint32_t step_seen = lift_step;
	lifted_next.clear();
	auto visit = [&](const JPH::BodyID &id) {
		JPH::Body *jolt_body = bodies.TryGetBody(id);
		if (jolt_body == nullptr || !jolt_body->IsDynamic()) {
			return;
		}

		JoltBody3D *body = reinterpret_cast<JoltBody3D *>(jolt_body->GetUserData());
		if (body->lift_seen_step == step_seen) {
			return;
		}
		body->lift_seen_step = step_seen;
		_lift_mass(*jolt_body, *body, ratio, step, g);
	};
	for (const JPH::BodyID &id : lifted_bodies) {
		visit(id);
	}
	for (uint32_t i = 0; i < loaded; ++i) {
		visit(loaded_bodies[i]);
	}
	SWAP(lifted_bodies, lifted_next);

	// The next step's stamps (0 is never one) and a slot for every body Jolt can hold.
	lift_step = lift_step == UINT32_MAX ? 1 : lift_step + 1;
	loaded_count.store(0, std::memory_order_relaxed);
	if (ratio > 0.0f && loaded_bodies.size() < system.GetMaxBodies()) {
		loaded_bodies.resize(system.GetMaxBodies());
	}
}

// One body's step of _lift_masses; keeps it in the lifted list while it is lifted
// or still has a load.
void JoltContactListener3D::_lift_mass(JPH::Body &p_jolt_body, JoltBody3D &p_body, float p_ratio, float p_step, float p_gravity) {
	constexpr float LOAD_SMOOTHING = 0.25f;
	const JPH::BodyID id = p_jolt_body.GetID();
	const float load_impulse = p_body.resting_load_impulse.exchange(0.0f, std::memory_order_relaxed);
	const float cap = p_body.resting_load_cap.exchange(0.0f, std::memory_order_relaxed);
	const float anticipated = p_body.anticipated_load.exchange(0.0f, std::memory_order_relaxed);
	if (!p_jolt_body.IsActive()) {
		if (p_body.mass_lifted || p_body.resting_load > 0.0f) {
			lifted_next.push_back(id);
		}
		return;
	}

	const float true_mass = p_body.get_mass();
	JPH::MotionProperties *motion = p_jolt_body.GetMotionPropertiesUnchecked();
	const float current = 1.0f / motion->GetInverseMass();

	// The measured load, no more than the heaviest body resting on this one: the
	// solver's trouble is the mass ratio between two touching bodies, not the load.
	// Uncapped, the bottom of three 1 kg boxes and a ragdoll's legs were lifted, and
	// a hand pressing a stick onto a part lifted the part, differently on each peer
	// of a networked game.
	const float load = (p_ratio > 0.0f && p_step > 0.0f && p_gravity > 0.0f) ? MIN(load_impulse / (p_step * p_gravity), cap) : 0.0f;
	p_body.resting_load += (load - p_body.resting_load) * LOAD_SMOOTHING;
	// An expected load (_note_resting_load) counts at once, past the smoothing and
	// the rate limit.
	p_body.resting_load = MAX(p_body.resting_load, anticipated);
	if (p_body.resting_load < true_mass * 0.01f) {
		p_body.resting_load = 0.0f;
	}

	const float target = p_ratio > 0.0f ? MAX(true_mass, p_body.resting_load / (p_ratio * LIFT_MARGIN)) : true_mass;
	float next = CLAMP(target, current * 0.8f, current * 1.25f);
	if (anticipated > 0.0f) {
		next = MAX(next, target);
	}
	if (next <= true_mass * 1.001f) {
		if (p_body.mass_lifted) {
			motion->ScaleToMass(true_mass);
			p_body.mass_lifted = false;
		}
		if (p_body.resting_load > 0.0f) {
			lifted_next.push_back(id);
		}
		return;
	}

	motion->ScaleToMass(next);
	if (next > current) {
		// Momentum kept: velocities scale down with the mass.
		const float keep = current / next;
		motion->SetLinearVelocity(motion->GetLinearVelocity() * keep);
		motion->SetAngularVelocity(motion->GetAngularVelocity() * keep);
	}
	p_body.mass_lifted = true;
	lifted_next.push_back(id);
}

bool JoltContactListener3D::_try_apply_surface_velocities(const JPH::Body &p_jolt_body1, const JPH::Body &p_jolt_body2, JPH::ContactSettings &p_settings) {
	if (p_jolt_body1.IsSensor() || p_jolt_body2.IsSensor()) {
		return false;
	}

	const bool supports_surface_velocity1 = !p_jolt_body1.IsDynamic();
	const bool supports_surface_velocity2 = !p_jolt_body2.IsDynamic();

	if (supports_surface_velocity1 == supports_surface_velocity2) {
		return false;
	}

	const JoltBody3D *body1 = reinterpret_cast<JoltBody3D *>(p_jolt_body1.GetUserData());
	const JoltBody3D *body2 = reinterpret_cast<JoltBody3D *>(p_jolt_body2.GetUserData());

	const bool has_surface_velocity1 = supports_surface_velocity1 && (body1->get_linear_surface_velocity() != Vector3() || body1->get_angular_surface_velocity() != Vector3());
	const bool has_surface_velocity2 = supports_surface_velocity2 && (body2->get_linear_surface_velocity() != Vector3() || body2->get_angular_surface_velocity() != Vector3());

	if (has_surface_velocity1 == has_surface_velocity2) {
		return false;
	}

	const JPH::Vec3 linear_velocity1 = to_jolt(body1->get_linear_surface_velocity());
	const JPH::Vec3 angular_velocity1 = to_jolt(body1->get_angular_surface_velocity());

	const JPH::Vec3 linear_velocity2 = to_jolt(body2->get_linear_surface_velocity());
	const JPH::Vec3 angular_velocity2 = to_jolt(body2->get_angular_surface_velocity());

	const JPH::RVec3 com1 = p_jolt_body1.GetCenterOfMassPosition();
	const JPH::RVec3 com2 = p_jolt_body2.GetCenterOfMassPosition();
	const JPH::Vec3 rel_com2 = JPH::Vec3(com2 - com1);

	const JPH::Vec3 angular_linear_velocity2 = rel_com2.Cross(angular_velocity2);
	const JPH::Vec3 total_linear_velocity2 = linear_velocity2 + angular_linear_velocity2;

	p_settings.mRelativeLinearSurfaceVelocity = total_linear_velocity2 - linear_velocity1;
	p_settings.mRelativeAngularSurfaceVelocity = angular_velocity2 - angular_velocity1;

	return true;
}

bool JoltContactListener3D::_try_add_contacts(const JPH::Body &p_jolt_body1, const JPH::Body &p_jolt_body2, const JPH::ContactManifold &p_manifold, JPH::ContactSettings &p_settings) {
	if (p_jolt_body1.IsSensor() || p_jolt_body2.IsSensor()) {
		return false;
	}

	const JoltBody3D *body1 = reinterpret_cast<JoltBody3D *>(p_jolt_body1.GetUserData());
	const JoltBody3D *body2 = reinterpret_cast<JoltBody3D *>(p_jolt_body2.GetUserData());

	if (!body1->reports_contacts() && !body2->reports_contacts()) {
		return false;
	}

	const JPH::SubShapeIDPair shape_pair(p_jolt_body1.GetID(), p_manifold.mSubShapeID1, p_jolt_body2.GetID(), p_manifold.mSubShapeID2);

	Manifold &manifold = [&]() -> Manifold & {
		const MutexLock write_lock(write_mutex);
		Manifold *existing = manifolds_by_shape_pair.getptr(shape_pair);
		if (existing != nullptr) {
			return *existing;
		}
		if (JoltProjectSettings::ordered_contact_reports) {
			new_shape_pairs.push_back(shape_pair);
		}
		return manifolds_by_shape_pair.insert(shape_pair, Manifold())->value;
	}();

	if (unlikely(!manifold.contacts1.is_empty())) {
		// CCD collisions can result in two contact callbacks for the same shape pair, one in the earlier discrete stage and one in the later CCD stage.
		// We want the manifolds from the discrete stage, as the bodies still have their original velocities at that point, so we early-out if we've already stored something.
		return false;
	}

	const JPH::uint contact_count = p_manifold.mRelativeContactPointsOn1.size();

	manifold.contacts1.reserve((uint32_t)contact_count);
	manifold.contacts2.reserve((uint32_t)contact_count);
	manifold.depth = p_manifold.mPenetrationDepth;

	JPH::CollisionEstimationResult collision;
	JPH::EstimateCollisionResponse(p_jolt_body1, p_jolt_body2, p_manifold, collision, p_settings.mCombinedFriction, p_settings.mCombinedRestitution, JoltProjectSettings::bounce_velocity_threshold, 5);

	for (JPH::uint i = 0; i < contact_count; ++i) {
		const JPH::RVec3 relative_point1 = JPH::RVec3(p_manifold.mRelativeContactPointsOn1[i]);
		const JPH::RVec3 relative_point2 = JPH::RVec3(p_manifold.mRelativeContactPointsOn2[i]);

		const JPH::RVec3 world_point1 = p_manifold.mBaseOffset + relative_point1;
		const JPH::RVec3 world_point2 = p_manifold.mBaseOffset + relative_point2;

		const JPH::Vec3 velocity1 = p_jolt_body1.GetPointVelocity(world_point1);
		const JPH::Vec3 velocity2 = p_jolt_body2.GetPointVelocity(world_point2);

		const JPH::CollisionEstimationResult::Impulse &impulse = collision.mImpulses[i];

		const JPH::Vec3 contact_impulse = p_manifold.mWorldSpaceNormal * impulse.mContactImpulse;
		const JPH::Vec3 friction_impulse1 = collision.mTangent1 * impulse.mFrictionImpulse1;
		const JPH::Vec3 friction_impulse2 = collision.mTangent2 * impulse.mFrictionImpulse2;
		const JPH::Vec3 combined_impulse = contact_impulse + friction_impulse1 + friction_impulse2;

		Contact contact1;
		contact1.point_self = to_godot(world_point1);
		contact1.point_other = to_godot(world_point2);
		contact1.normal = to_godot(-p_manifold.mWorldSpaceNormal);
		contact1.velocity_self = to_godot(velocity1);
		contact1.velocity_other = to_godot(velocity2);
		contact1.impulse = to_godot(-combined_impulse);
		manifold.contacts1.push_back(contact1);

		Contact contact2;
		contact2.point_self = to_godot(world_point2);
		contact2.point_other = to_godot(world_point1);
		contact2.normal = to_godot(p_manifold.mWorldSpaceNormal);
		contact2.velocity_self = to_godot(velocity2);
		contact2.velocity_other = to_godot(velocity1);
		contact2.impulse = to_godot(combined_impulse);
		manifold.contacts2.push_back(contact2);
	}

	return true;
}

bool JoltContactListener3D::_try_evaluate_area_overlap(const JPH::Body &p_body1, const JPH::Body &p_body2, const JPH::SubShapeID &p_shape_id1, const JPH::SubShapeID &p_shape_id2) {
	if (!p_body1.IsSensor() && !p_body2.IsSensor()) {
		return false;
	}

	const JPH::SubShapeIDPair shape_pair1(p_body1.GetID(), p_shape_id1, p_body2.GetID(), p_shape_id2);
	const JPH::SubShapeIDPair shape_pair2(p_body2.GetID(), p_shape_id2, p_body1.GetID(), p_shape_id1);

	const JoltObject3D *object1 = reinterpret_cast<JoltObject3D *>(p_body1.GetUserData());
	const JoltObject3D *object2 = reinterpret_cast<JoltObject3D *>(p_body2.GetUserData());

	const JoltArea3D *area1 = object1->as_area();
	const JoltArea3D *area2 = object2->as_area();

	const JoltBody3D *body1 = object1->as_body();
	const JoltBody3D *body2 = object2->as_body();

	const JoltSoftBody3D *soft_body1 = object1->as_soft_body();
	const JoltSoftBody3D *soft_body2 = object2->as_soft_body();

	if (area1 != nullptr && area2 != nullptr) {
		_evaluate_area_overlap(*area1, *area2, shape_pair1);
		_evaluate_area_overlap(*area2, *area1, shape_pair2);
	} else if (area1 != nullptr && body2 != nullptr) {
		_evaluate_area_overlap(*area1, *body2, shape_pair1);
	} else if (area2 != nullptr && body1 != nullptr) {
		_evaluate_area_overlap(*area2, *body1, shape_pair2);
	} else if (area1 != nullptr && soft_body2 != nullptr) {
		_evaluate_area_overlap(*area1, *soft_body2, shape_pair1);
	} else if (area2 != nullptr && soft_body1 != nullptr) {
		_evaluate_area_overlap(*area2, *soft_body1, shape_pair2);
	}

	return true;
}

bool JoltContactListener3D::_try_remove_contacts(const JPH::SubShapeIDPair &p_shape_pair) {
	const MutexLock write_lock(write_mutex);
	return manifolds_by_shape_pair.erase(p_shape_pair);
}

bool JoltContactListener3D::_try_remove_area_overlap(const JPH::SubShapeIDPair &p_shape_pair) {
	const JPH::SubShapeIDPair swapped_shape_pair(p_shape_pair.GetBody2ID(), p_shape_pair.GetSubShapeID2(), p_shape_pair.GetBody1ID(), p_shape_pair.GetSubShapeID1());

	const MutexLock write_lock(write_mutex);

	bool removed = false;

	if (area_overlaps.erase(p_shape_pair)) {
		area_exits.insert(p_shape_pair);
		removed = true;
	}

	if (area_overlaps.erase(swapped_shape_pair)) {
		area_exits.insert(swapped_shape_pair);
		removed = true;
	}

	return removed;
}

#ifdef DEBUG_ENABLED

bool JoltContactListener3D::_try_add_debug_contacts(const JPH::Body &p_body1, const JPH::Body &p_body2, const JPH::ContactManifold &p_manifold) {
	if (p_body1.IsSensor() || p_body2.IsSensor()) {
		return false;
	}

	const int64_t max_count = debug_contacts.size();

	if (max_count == 0) {
		return false;
	}

	const int additional_pairs = (int)p_manifold.mRelativeContactPointsOn1.size();
	const int additional_contacts = additional_pairs * 2;

	int current_count = debug_contact_count.load(std::memory_order_relaxed);
	bool exchanged = false;

	do {
		const int new_count = current_count + additional_contacts;

		if (new_count > max_count) {
			return false;
		}

		exchanged = debug_contact_count.compare_exchange_weak(current_count, new_count, std::memory_order_release, std::memory_order_relaxed);
	} while (!exchanged);

	for (int i = 0; i < additional_pairs; ++i) {
		const int pair_index = current_count + i * 2;

		const JPH::RVec3 point_on_1 = p_manifold.GetWorldSpaceContactPointOn1((JPH::uint)i);
		const JPH::RVec3 point_on_2 = p_manifold.GetWorldSpaceContactPointOn2((JPH::uint)i);

		debug_contacts.write[pair_index + 0] = to_godot(point_on_1);
		debug_contacts.write[pair_index + 1] = to_godot(point_on_2);
	}

	return true;
}

bool JoltContactListener3D::_try_add_debug_contacts(const JPH::Body &p_soft_body, const JPH::SoftBodyManifold &p_manifold) {
	const int64_t max_count = debug_contacts.size();
	if (max_count == 0) {
		return false;
	}

	int additional_contacts = 0;

	for (const JPH::SoftBodyVertex &vertex : p_manifold.GetVertices()) {
		if (p_manifold.HasContact(vertex)) {
			additional_contacts += 1;
		}
	}

	int current_count = debug_contact_count.load(std::memory_order_relaxed);
	bool exchanged = false;

	do {
		const int new_count = current_count + additional_contacts;

		if (new_count > max_count) {
			return false;
		}

		exchanged = debug_contact_count.compare_exchange_weak(current_count, new_count, std::memory_order_release, std::memory_order_relaxed);
	} while (!exchanged);

	int contact_index = current_count;

	for (const JPH::SoftBodyVertex &vertex : p_manifold.GetVertices()) {
		if (!p_manifold.HasContact(vertex)) {
			continue;
		}

		const JPH::RMat44 body_com_transform = p_soft_body.GetCenterOfMassTransform();
		const JPH::Vec3 local_contact_point = p_manifold.GetLocalContactPoint(vertex);
		const JPH::RVec3 contact_point = body_com_transform * local_contact_point;

		debug_contacts.write[contact_index++] = to_godot(contact_point);
	}

	return true;
}

#endif

bool JoltContactListener3D::_has_shape_shifted(const JoltShapedObject3D &p_object, const JPH::SubShapeID &p_sub_shape_id) {
	return p_object.get_previous_jolt_shape() != nullptr && p_object.get_jolt_shape()->GetSubShapeUserData(p_sub_shape_id) != p_object.get_previous_jolt_shape()->GetSubShapeUserData(p_sub_shape_id);
}

void JoltContactListener3D::_evaluate_area_overlap(const JoltArea3D &p_area, const JoltArea3D &p_other_area, const JPH::SubShapeIDPair &p_shape_pair) {
	const MutexLock write_lock(write_mutex);

	if (p_area.can_monitor(p_other_area)) {
		if (!area_overlaps.has(p_shape_pair)) {
			area_overlaps.insert(p_shape_pair);
			area_enters.insert(p_shape_pair);
		} else if (_has_shape_shifted(p_area, p_shape_pair.GetSubShapeID1()) || _has_shape_shifted(p_other_area, p_shape_pair.GetSubShapeID2())) {
			// A shape has taken on the `JPH::SubShapeID` value of another shape, likely because of the other shape having been replaced or moved
			// in some way, so we force the area to refresh its internal mappings by exiting and entering this shape pair.
			area_exits.insert(p_shape_pair);
			area_enters.insert(p_shape_pair);
		}
	} else {
		if (area_overlaps.erase(p_shape_pair)) {
			area_exits.insert(p_shape_pair);
		}
	}
}

void JoltContactListener3D::_evaluate_area_overlap(const JoltArea3D &p_area, const JoltBody3D &p_body, const JPH::SubShapeIDPair &p_shape_pair) {
	const MutexLock write_lock(write_mutex);

	if (p_area.can_monitor(p_body)) {
		if (!area_overlaps.has(p_shape_pair)) {
			area_overlaps.insert(p_shape_pair);
			area_enters.insert(p_shape_pair);
		} else if (_has_shape_shifted(p_area, p_shape_pair.GetSubShapeID1()) || _has_shape_shifted(p_body, p_shape_pair.GetSubShapeID2())) {
			// A shape has taken on the `JPH::SubShapeID` value of another shape, likely because of the other shape having been replaced or moved
			// in some way, so we force the area to refresh its internal mappings by exiting and entering this shape pair.
			area_exits.insert(p_shape_pair);
			area_enters.insert(p_shape_pair);
		}
	} else {
		if (area_overlaps.erase(p_shape_pair)) {
			area_exits.insert(p_shape_pair);
		}
	}
}

void JoltContactListener3D::_evaluate_area_overlap(const JoltArea3D &p_area, const JoltSoftBody3D &p_body, const JPH::SubShapeIDPair &p_shape_pair) {
	const MutexLock write_lock(write_mutex);

	if (p_area.can_monitor(p_body)) {
		area_soft_body_overlaps.push_back(p_shape_pair);
		if (!area_exits.erase(p_shape_pair)) {
			area_enters.insert(p_shape_pair);
		}
	}
}

void JoltContactListener3D::_flush_contacts() {
	// DwarvenIndustry: ordered contact reports. The map keeps the pairs in the
	// order they were added, which is the order bodies get their contacts, and a
	// body reporting as many as it may keeps the first ones. Pairs that started
	// touching in this step were added in whatever order Jolt's worker threads
	// found them, so with the setting on they're moved to the end in key order:
	// the same contacts on every run.
	if (new_shape_pairs.size() > 1) {
		new_shape_pairs.sort();
		for (const JPH::SubShapeIDPair &shape_pair : new_shape_pairs) {
			Manifold *manifold = manifolds_by_shape_pair.getptr(shape_pair);
			if (manifold == nullptr) {
				continue;
			}
			Manifold moved = *manifold;
			manifolds_by_shape_pair.erase(shape_pair);
			manifolds_by_shape_pair.insert(shape_pair, moved);
		}
	}
	new_shape_pairs.clear();

	for (KeyValue<JPH::SubShapeIDPair, Manifold> &E : manifolds_by_shape_pair) {
		const JPH::SubShapeIDPair &shape_pair = E.key;
		Manifold &manifold = E.value;

		JoltBody3D *body1 = space->try_get_body(shape_pair.GetBody1ID());
		ERR_FAIL_NULL(body1);

		JoltBody3D *body2 = space->try_get_body(shape_pair.GetBody2ID());
		ERR_FAIL_NULL(body2);

		const int shape_index1 = body1->find_shape_index(shape_pair.GetSubShapeID1());
		const int shape_index2 = body2->find_shape_index(shape_pair.GetSubShapeID2());

		for (const Contact &contact : manifold.contacts1) {
			body1->add_contact(body2, manifold.depth, shape_index1, shape_index2, contact.normal, contact.point_self, contact.point_other, contact.velocity_self, contact.velocity_other, contact.impulse);
		}

		for (const Contact &contact : manifold.contacts2) {
			body2->add_contact(body1, manifold.depth, shape_index2, shape_index1, contact.normal, contact.point_self, contact.point_other, contact.velocity_self, contact.velocity_other, contact.impulse);
		}

		manifold.contacts1.clear();
		manifold.contacts2.clear();
	}
}

void JoltContactListener3D::_flush_area_enters() {
	for (const JPH::SubShapeIDPair &shape_pair : area_enters) {
		const JPH::BodyID &body_id1 = shape_pair.GetBody1ID();
		const JPH::BodyID &body_id2 = shape_pair.GetBody2ID();

		JoltObject3D *object1 = space->try_get_object(body_id1);
		JoltObject3D *object2 = space->try_get_object(body_id2);

		if (object1 == nullptr || object2 == nullptr) {
			continue;
		}

		JoltArea3D *area1 = object1->as_area();
		JoltArea3D *area2 = object2->as_area();

		const JPH::SubShapeID &sub_shape_id1 = shape_pair.GetSubShapeID1();
		const JPH::SubShapeID &sub_shape_id2 = shape_pair.GetSubShapeID2();

		if (area1 != nullptr && area2 != nullptr) {
			area1->area_shape_entered(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (area1 != nullptr && area2 == nullptr) {
			area1->body_shape_entered(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (area1 == nullptr && area2 != nullptr) {
			area2->body_shape_entered(body_id1, sub_shape_id1, sub_shape_id2);
		}
	}

	area_enters.clear();
}

void JoltContactListener3D::_flush_area_exits() {
	for (const JPH::SubShapeIDPair &shape_pair : area_exits) {
		const JPH::BodyID &body_id1 = shape_pair.GetBody1ID();
		const JPH::BodyID &body_id2 = shape_pair.GetBody2ID();

		JoltObject3D *object1 = space->try_get_object(body_id1);
		JoltObject3D *object2 = space->try_get_object(body_id2);

		JoltArea3D *area1 = object1 != nullptr ? object1->as_area() : nullptr;
		JoltArea3D *area2 = object2 != nullptr ? object2->as_area() : nullptr;

		const JoltBody3D *body1 = object1 != nullptr ? object1->as_body() : nullptr;
		const JoltBody3D *body2 = object2 != nullptr ? object2->as_body() : nullptr;

		const JPH::SubShapeID &sub_shape_id1 = shape_pair.GetSubShapeID1();
		const JPH::SubShapeID &sub_shape_id2 = shape_pair.GetSubShapeID2();

		if (area1 != nullptr && area2 != nullptr) {
			area1->area_shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (area1 != nullptr && body2 != nullptr) {
			area1->body_shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (body1 != nullptr && area2 != nullptr) {
			area2->body_shape_exited(body_id1, sub_shape_id1, sub_shape_id2);
		} else if (area1 != nullptr) {
			area1->shape_exited(body_id2, sub_shape_id2, sub_shape_id1);
		} else if (area2 != nullptr) {
			area2->shape_exited(body_id1, sub_shape_id1, sub_shape_id2);
		}
	}

	area_exits.clear();
}

void JoltContactListener3D::_clear_area_soft_body_overlaps() {
	area_exits.reserve(area_soft_body_overlaps.size());
	for (const JPH::SubShapeIDPair &shape_pair : area_soft_body_overlaps) {
		area_exits.insert(shape_pair);
	}

	area_soft_body_overlaps.clear();
}

void JoltContactListener3D::pre_step() {
	_clear_area_soft_body_overlaps();

	_lift_masses();

#ifdef DEBUG_ENABLED
	debug_contact_count = 0;
#endif
}

void JoltContactListener3D::post_step() {
	_flush_contacts();
	_flush_area_exits();
	_flush_area_enters();
}
