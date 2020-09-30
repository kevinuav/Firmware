/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file loiter.cpp
 *
 * Helper class to loiter
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "loiter.h"
#include "navigator.h"

Loiter::Loiter(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
Loiter::on_inactive()
{
	_loiter_pos_set = false;
	_seted = false;
}

void
Loiter::on_activation()
{
	if (_navigator->get_reposition_triplet()->current.valid) {
		reposition();

	} else {
		set_loiter_position();
	}
}

void
Loiter::on_active()
{
	if (_navigator->get_reposition_triplet()->current.valid) {
		reposition();
	}
	if(_local_pos_sub.updated()){     //读取飞机对地速度
		_local_pos_sub.copy(&_local_position);
	}
	get_vector_to_next_waypoint(_navigator->get_global_position()->lat,_navigator->get_global_position()->lon
					,_target_pos.lat,_target_pos.lon,&_vn,&_ve);

	float angle=angleA2B(_local_position.vx,_local_position.vy,_vn,_ve);
	struct position_setpoint_triplet_s *cur_sp = _navigator->get_reposition_triplet();
//	warnx("angle=%f",(double)angle);

	if(angle<(float)20.0 && (!_seted))
	{
	wind_drift(_windf,_navigator->get_global_position()->alt,&_driftn,&_drifte);
	_air_lat = _target_pos.lat-_driftn;
	_air_lon = _target_pos.lon-_drifte;
	create_waypoint_from_line_and_dist(_air_lat,_air_lon,_navigator->get_global_position()->lat,_navigator->get_global_position()->lon,
						-50.0,&_mission_item.lat,&_mission_item.lon);

	_loiter_pos_set = false;

	warnx("prelat=%f prelon=%f newlat=%f newlon=%f",_air_lat,_air_lon,_mission_item.lat,_mission_item.lon);
	cur_sp->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_seted = true;
	}
	// reset the loiter position if we get disarmed
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		_loiter_pos_set = false;
	}

	warnx("reach=%d",(int)_waypoint_position_reached);

	if(is_mission_item_reached())
	{
		cur_sp->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	}
}

void
Loiter::set_loiter_position()
{
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED &&
	    _navigator->get_land_detected()->landed) {

		// Not setting loiter position if disarmed and landed, instead mark the current
		// setpoint as invalid and idle (both, just to be sure).

		_navigator->set_can_loiter_at_sp(false);
		_navigator->get_position_setpoint_triplet()->current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		_navigator->set_position_setpoint_triplet_updated();
		_loiter_pos_set = false;
		return;

	} else if (_loiter_pos_set) {
		// Already set, nothing to do.
		return;
	}

	_loiter_pos_set = true;

	// set current mission item to loiter
	set_loiter_item(&_mission_item, _navigator->get_loiter_min_alt());

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.velocity_valid = false;
	pos_sp_triplet->previous.valid = false;
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);
	_navigator->set_position_setpoint_triplet_updated();
}

void
Loiter::reposition()
{
	// we can't reposition if we are not armed yet
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}
	struct position_setpoint_triplet_s *rep = _navigator->get_reposition_triplet();




	if (rep->current.valid) {
		// set loiter position based on reposition command

		_target_pos.lat = rep->current.lat;
		_target_pos.lon = rep->current.lon;

		// convert mission item to current setpoint
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.velocity_valid = false;
		pos_sp_triplet->previous.yaw = _navigator->get_local_position()->heading;
		pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
		memcpy(&pos_sp_triplet->current, &rep->current, sizeof(rep->current));
		pos_sp_triplet->next.valid = false;
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);
		_navigator->set_position_setpoint_triplet_updated();

		// mark this as done
		memset(rep, 0, sizeof(*rep));
		_seted = false;
		_waypoint_position_reached = false;

	}
}
