/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "state_machine.h"

namespace state_machine
{
void StateTrafficLightStop::update(StateContext *context)
{
  if (context->getLightColor() == TrafficLight::GREEN)
    context->setState(StateMoveForward::create());
}

void StateMoveForward::update(StateContext *context)
{
  if (context->getLightColor() == TrafficLight::RED)
    context->setState(StateTrafficLightStop::create());

  if(context->getChangeFlag() == ChangeFlag::right || context->getChangeFlag() == ChangeFlag::left)
    context->setState(StateLaneChange::create());
}

void StateLaneChange::update(StateContext *context)
{
  if(context->getChangeFlag() == ChangeFlag::straight)
    context->setState(StateMoveForward::create());
}

void StateStopSignStop::update(StateContext *context)
{
  // stop sign stop
}

void StateMissionComplete::update(StateContext *context)
{
  // Mission complete
}

void StateEmergency::update(StateContext *context)
{
  // Emergency
}

void StateObstacleAvoidance::update(StateContext *context)
{
  // Obstacle Avoidance
}



}  // state_machine