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
 * @file ActuatorEffectivenessMCTilt2DOF.cpp
 *
 * Actuator effectiveness computed x, y and z components of 2dof thrust vectoring tiltrotor
 *
 * @author Håvard Brenne <hava.brenne@gmail.com>
 */
#include "ActuatorEffectivenessMCTilt2DOF.hpp"

using namespace matrix;

ActuatorEffectivenessMCTilt2DOF::ActuatorEffectivenessMCTilt2DOF(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this)
{
	for (int i = 0; i < ActuatorEffectivenessRotors::NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV_2D%u_EMAXA", i);
		_param_elev_max[i] = param_find(buffer);
	}
}

void ActuatorEffectivenessMCTilt2DOF::updateParams()
{
	ModuleParams::updateParams();

	// Not sure if _mc_rotors are already updated
	for (int i = 0; i < _mc_rotors.geometry().num_rotors; ++i) {

		param_get(_param_elev_max[i], &_max_elevation_angle[i]);
	}

}

bool
ActuatorEffectivenessMCTilt2DOF::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	const bool rotors_added_successfully = addActuators(configuration);

	return rotors_added_successfully;
}

bool
ActuatorEffectivenessMCTilt2DOF::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}


	const ActuatorEffectivenessRotors::Geometry &geometry = _mc_rotors.geometry();
	int num_actuators = computeEffectivenessMatrix(geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);

	// 2 servos per xyz decomposed rotor
	for (int i = 0; i < num_actuators*2/3; i++) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	return true;
}
int
ActuatorEffectivenessMCTilt2DOF::computeEffectivenessMatrix(const ActuatorEffectivenessRotors::Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;
	for (int i = 0; i < geometry.num_rotors; i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}
		//++num_actuators;
		num_actuators += 3;
		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		//Get thrust coefficient, used to scale u between 0 and 1. This will correspond to the max force.
		// Needs to be set to the same value in the thrust_vector_controller receiving u.
		float ct = geometry.rotors[i].thrust_coef;

		// Make sure km is given in terms of thrust scaling
		float km = ct*geometry.rotors[i].moment_ratio;

		// Fill corresponding items in effectiveness matrix
		// Decomposed forces
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j+3, 3*i + actuator_start_index + j) = ct;
			}
		// Decomposed moments
		effectiveness(0, 3*i + actuator_start_index) = 0;//Have to find seperate value;
		effectiveness(1, 3*i + actuator_start_index) = ct*position(2);
		effectiveness(2, 3*i + actuator_start_index) = -ct*position(1);
		effectiveness(0, 3*i + 1 + actuator_start_index) = -ct*position(2);
		effectiveness(1, 3*i + 1 + actuator_start_index) = 0;//km;
		effectiveness(2, 3*i + 1 + actuator_start_index) = ct*position(0);
		effectiveness(0, 3*i + 2 + actuator_start_index) = ct*position(1);
		effectiveness(1, 3*i + 2 + actuator_start_index) = -ct*position(0);
		effectiveness(2, 3*i + 2 + actuator_start_index) = km;
		}
	return num_actuators;
}

void ActuatorEffectivenessMCTilt2DOF::transformActuatorControls(actuator_motors_s &actuator_motors, actuator_servos_s &actuator_servos) {
	// variable change, incoming actuator_sp is the output in body frame force xyz components for each rotor
	// Need to convert actuator_sp to thrust magnitude and tilt azimuth and elevation angles

	// Loop for each rotor
	int num_rotors = _mc_rotors.geometry().num_rotors;
	for (int i = 0; i < num_rotors; i++) {
		Vector3f thrust_vector(actuator_motors.control[3*i], actuator_motors.control[3*i+1], actuator_motors.control[3*i+2]);
		//PX4_INFO("thrust_vector %d: %f, %f, %f", i,(double)thrust_vector(0), (double)thrust_vector(1), (double)thrust_vector(2));


		float thrust_magnitude = thrust_vector.norm();

		// Get thrust azimuth and elevation angles
		float thrust_azimuth = atan2f(thrust_vector(1), thrust_vector(0));
		float thrust_elevation = atan2f(sqrtf(thrust_vector(0) * thrust_vector(0) + thrust_vector(1) * thrust_vector(1)), -thrust_vector(2));

		// Set actuator setpoints. Map from min to max. Thrust and azimuth are guarantueed to be within bounds
		actuator_motors.control[i] = thrust_magnitude;

		//TODO: Handle this in saturation. With saturation flags?
		if  (thrust_elevation > (_max_elevation_angle[i]*M_PI_F/180)){
			actuator_servos.control[i] = 1;
		} else {
		actuator_servos.control[i] = 2*thrust_elevation/(_max_elevation_angle[i]*M_PI_F/180) -1;
		}
		actuator_servos.control[i + num_rotors] = thrust_azimuth/M_PI_F;

		//PX4_INFO("Servo %d: Elevation: %f, Azimuth: %f", i, (double)actuator_servos.control[i], (double)actuator_servos.control[i + num_rotors]);


		// Log thrust magnitude, azimuth and elevation angles
		/*PX4_INFO("thrust_magnitude %d: %f", i,(double)thrust_magnitude);
		PX4_INFO("thrust_azimuth %d: %f", i,(double)(thrust_azimuth*180.0f/M_PI_F));
		PX4_INFO("thrust_elevation %d: %f", i,(double)(thrust_elevation*180.0f/M_PI_F));*/

	}
}
