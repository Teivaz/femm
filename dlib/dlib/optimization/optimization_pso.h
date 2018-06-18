// Copyright (C) 2018  Oleksandr Kuznietsov (github@teivaz.com)
// License: Boost Software License   See LICENSE.txt for the full license.
#ifndef DLIB_OPTIMIZATIOn_PSO_H_
#define DLIB_OPTIMIZATIOn_PSO_H_

#include <cmath>
#include <limits>
#include "../matrix.h"
#include "../algs.h"
#include "optimization_search_strategies_abstract.h"
#include "../sequence.h"

namespace dlib
{

// ----------------------------------------------------------------------------------------

	struct pso_search_params {
		pso_search_params()
			:agents(50)
			, generations(10)
		{ max_iterations = agents * generations; }
		int agents = 50;
		int generations = 10;
		int max_iterations;
	};

	struct pso_stop_generations {
		pso_stop_generations(int gen = 10) : generations(gen) {}

		bool should_continue(int computation_step, const pso_search_params& p, double, double) const {
			return computation_step < p.max_iterations;
		}

		int generations;
	};

	template<typename T>
	struct pso_agent {
		pso_agent(const T& position, const T& velocity, double value) 
			: current_position(position)
			, current_velocity(velocity)
			, local_best_position(position)
			, local_best_value(value) 
		{}

		const T& calculate_next_position(const T& global_best_position, double constuction_k) {
			const double phi_max = 4;
			const double chi = constuction_k;

			const double rand_max = RAND_MAX;
			const double phi1 = std::rand() / rand_max * phi_max / 2;
			const double phi2 = std::rand() / rand_max * phi_max / 2;
			const double phi = phi1 + phi2;
			const T v_next = current_velocity + phi1 * (local_best_position - current_position) + phi2 * (global_best_position - current_position);
			const T x_next = chi * v_next + chi * current_position + (1 - chi) * (phi1 * local_best_position + phi2 * global_best_position) / (phi1 + phi2);
			current_velocity = v_next;
			current_position = x_next;
			return current_position;
		}
		T current_position;
		T current_velocity;
		T local_best_position;
		double local_best_value;
	};

	/*
	avg_step - desired step for every variable
	*/


	template <
		typename funct,
		typename funct_bounds,
		typename T
	>
	double pso_find_min(
		const pso_search_params& parameters,
		const funct& f_fit,
		const funct_bounds& f_bounds,
		T& x,
		const T& avg_step
	)
	{
		double best_value = f_fit(x);
		T best_position = x;
		int computation_step = 0;
		const double total_steps = parameters.max_iterations;
		const pso_stop_generations stop_strategy(parameters.generations);

		std::vector<pso_agent<T>> agents;
		agents.reserve(parameters.agents);
		// Populate agents
		for (int i = 0; i < parameters.agents; i++) {
			const T random_dir_coeff = randm(avg_step.nr(), avg_step.nc()) * 2.0 - 1.0;
			const T random_pos = randm(avg_step.nr(), avg_step.nc()) * 2.0 - 1.0;
			T velocity = pointwise_multiply(avg_step, random_dir_coeff);
			T position = best_position + pointwise_multiply(avg_step, random_dir_coeff) * 10;
			agents.push_back(pso_agent<T>(position, velocity, std::numeric_limits<double>::max()));
		}

		while (true) {
			double f = std::numeric_limits<double>::max();
			const double constriction_k = (computation_step + 1) / total_steps;
			const int agent_index = computation_step % parameters.agents;
			pso_agent<T>& agent = agents[agent_index];
			bool should_continue = true;

			const T& current_position = agent.calculate_next_position(best_position, constriction_k);
			if (f_bounds(current_position)) {
				f = f_fit(current_position);
				should_continue = stop_strategy.should_continue(computation_step, parameters, f, best_value);

				// Update best positions
				if (f < best_value) {
					best_value = f;
					best_position = current_position;
				}

				if (f < agent.local_best_value) {
					agent.local_best_value = f;
					agent.local_best_position = current_position;
				}
			}
			if (!should_continue) {
				break;
			}

			computation_step++;
		}

		return best_value;
	}

}

#endif // DLIB_OPTIMIZATIOn_PSO_H_

