//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/crash_course/mAddNoise.cpp
 *
 * \author  Javier Alberto Carrasco Melo
 *
 * \date    2022-02-10
 *
 */
//----------------------------------------------------------------------
#include "projects/crash_course/mAddNoise.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace crash_course
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mAddNoise> cCREATE_ACTION_FOR_M_ADDNOISE("AddNoise");
#endif

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mAddNoise constructor
//----------------------------------------------------------------------
mAddNoise::mAddNoise(core::tFrameworkElement *parent, const std::string &name)
    : tModule(parent, name, false),
    standard_deviation(0.05),
    normal_distribution(0, 0.05),
    eng(1234)
{}

//----------------------------------------------------------------------
// mAddNoise destructor
//----------------------------------------------------------------------
mAddNoise::~mAddNoise()
{}

//----------------------------------------------------------------------
// mAddNoise OnParameterChange
//----------------------------------------------------------------------
void mAddNoise::OnParameterChange()
{
  normal_distribution = std::normal_distribution<double>(0, standard_deviation.Get().Value());
}
//----------------------------------------------------------------------
// mAddNoise Update
//----------------------------------------------------------------------
void mAddNoise::Update()
{
  output.Publish(input.Get() + rrlib::si_units::tLength<>(normal_distribution(eng)));
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
