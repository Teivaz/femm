#pragma once

struct lua_State;

void initalise_dlib_lua(lua_State* lua);

namespace dlib_lua {
	/*
	 * Having:
	 * fitting_fun( {value} ) -> number
	 * fitting_fun_derivative( {value} ) -> {derivative}
	 * lower_bound { min_value }
	 * upper_bound { max_value }
	 * initial_value { value }
	 * where 'min_value', 'max_value', 'value', and 'derivative' are arrays of numbers of same size
	 *
	 * op_bfgs(fitting_fun, fitting_fun_derivative, lower_bound, upper_bound, initial_value) -> result_at_found_value, { value }
	 */
	int bfgs(lua_State* lua);

	/*
	 * Having:
	 * fitting_fun( {value} ) -> number
	 * lower_bound { min_value }
	 * upper_bound { max_value }
	 * initial_value { value }
	 * where 'min_value', 'max_value', 'value', and 'derivative' are arrays of numbers of same size
	 *
	 * op_pso(fitting_fun, lower_bound, upper_bound, initial_value) -> result_at_found_value, { value }
	 *
	 */
	int pso(lua_State* lua);
}
