

/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessMCTilt2DOF.hpp
 *
 * Actuator effectiveness computed x, y and z components of 2dof thrust vectoring rotorcraft
 * @author Håvard Brenne <hava.brenne@gmail.com>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

class ActuatorEffectivenessMCTilt2DOF : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessMCTilt2DOF(ModuleParams *parent);
	virtual ~ActuatorEffectivenessMCTilt2DOF() = default;
	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;
	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}
	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}
	bool addActuators(Configuration &configuration);
	static int computeEffectivenessMatrix(const ActuatorEffectivenessRotors::Geometry &geometry,
					      EffectivenessMatrix &effectiveness, int actuator_start_index = 0);
	const char *name() const override { return "MCTilt2DOF"; }
	void transformActuatorControls(actuator_motors_s &actuator_motors, actuator_servos_s &actuator_servos) override;

private:
	void updateParams() override;
	ActuatorEffectivenessRotors _mc_rotors;
	param_t _param_elev_max[ActuatorEffectivenessRotors::NUM_ROTORS_MAX];
	float _max_elevation_angle[ActuatorEffectivenessRotors::NUM_ROTORS_MAX];
};
