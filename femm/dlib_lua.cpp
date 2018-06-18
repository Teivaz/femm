
#include "stdafx.h"
#include "dlib_lua.h"
#include "lua.h"

using namespace dlib;

using column_vector = matrix<double, 0, 1>;

void initalise_dlib_lua(lua_State* lua) {

	lua_register(lua, "op_bfgs", dlib_lua::bfgs);
	lua_register(lua, "op_pso", dlib_lua::pso);
}

column_vector lua_toarray(lua_State* lua, int index) {
	column_vector result;
	int count = lua_getn(lua, index);
	result.set_size(count);
	for (int i = 1; i <= count; i++) {
		lua_pushnumber(lua, i);
		lua_gettable(lua, index);
		result(i - 1) = lua_todouble(lua, -1);
		lua_pop(lua, 1);
	}
	return result;
}

void lua_pusharray(lua_State* lua, const column_vector& array) {
	lua_newtable(lua);
	int arg_idx = 1;
	for (double arg : array) {
		lua_pushnumber(lua, arg_idx);
		lua_pushnumber(lua, arg);
		lua_settable(lua, -3);
		arg_idx++;
	}
}

namespace dlib_lua {

	int bfgs(lua_State* lua) {
		double stop_delta = 1e-7;
		int max_iterations = 100;
		double derivative_epsilon = 1e-6;

		int argn = lua_gettop(lua);
		if (!lua_isfunction(lua, 1)) {
			// Error. Should be function
		}
		auto lower_bound = lua_toarray(lua, 2);
		auto upper_bound = lua_toarray(lua, 3);
		auto value = lua_toarray(lua, 4);
		if (argn == 5) {
			lua_pushstring(lua, "derivative_epsilon");
			lua_gettable(lua, 5);
			if (!lua_isnil(lua, -1)) derivative_epsilon = lua_todouble(lua, -1);
			lua_pop(lua, 1);
			lua_pushstring(lua, "max_iterations");
			lua_gettable(lua, 5);
			if (!lua_isnil(lua, -1)) max_iterations = lua_todouble(lua, -1);
			lua_pop(lua, 1);
			lua_pushstring(lua, "stop_delta");
			lua_gettable(lua, 5);
			if (!lua_isnil(lua, -1)) stop_delta = lua_todouble(lua, -1);
			lua_pop(lua, 1);
		}
		auto fun = [=](const column_vector& value) -> double {
			lua_pushvalue(lua, 1);
			lua_pusharray(lua, value);
			lua_call(lua, 1, 1);
			double result = lua_todouble(lua, 1);
			return result;
		};
		double f = find_min_using_approximate_derivatives(bfgs_search_strategy(), objective_delta_stop_strategy(stop_delta, max_iterations), fun, value, 0, derivative_epsilon);
		//double f = find_min_box_constrained(bfgs_search_strategy(), objective_delta_stop_strategy(1e-9), fun, fun_derivative, value, lower_bound, upper_bound);
		lua_pop(lua, argn);
		lua_pushnumber(lua, f);
		lua_pusharray(lua, value);
		return 2;
	}

	int pso(lua_State* lua) {
		double stop_delta = 1e-7;
		int max_iterations = 100;
		double derivative_epsilon = 1e-6;

		int argn = lua_gettop(lua);
		if (!lua_isfunction(lua, 1)) {
			// Error. Should be function
		}
		if (!lua_isfunction(lua, 2)) {
			// Error. Should be function
		}
		auto value = lua_toarray(lua, 3);
		auto step = lua_toarray(lua, 4);
		pso_search_params search_params;
		if (argn == 5) {
			lua_pushstring(lua, "agents");
			lua_gettable(lua, 5);
			if (!lua_isnil(lua, -1)) search_params.agents = lua_todouble(lua, -1);
			lua_pushstring(lua, "generations");
			lua_gettable(lua, 5);
			if (!lua_isnil(lua, -1)) search_params.generations = lua_todouble(lua, -1);
			lua_pop(lua, 1);
			lua_pushstring(lua, "max_iterations");
			lua_gettable(lua, 5);
			if (!lua_isnil(lua, -1)) search_params.max_iterations = lua_todouble(lua, -1);
			lua_pop(lua, 1);
		}

		auto fun = [=](const column_vector& value) -> double {
			lua_pushvalue(lua, 1);
			lua_pusharray(lua, value);
			lua_rawcall(lua, 1, 1);
			double result = lua_todouble(lua, -1);
			lua_pop(lua, 1);
			return result;
		};

		auto fun_bounds = [=](const column_vector& value) -> bool {
			lua_pushvalue(lua, 2);
			lua_pusharray(lua, value);
			lua_rawcall(lua, 1, 1);
			double result = lua_todouble(lua, -1);
			lua_pop(lua, 1);
			return result;
		};

		double f = pso_find_min(search_params, fun, fun_bounds, value, step);
		lua_pop(lua, argn);
		lua_pushnumber(lua, f);
		lua_pusharray(lua, value);
		return 2;
	}
}

