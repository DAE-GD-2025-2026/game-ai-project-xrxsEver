
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"
#include "DrawDebugHelpers.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior> &WeightedBehaviors)
	: WeightedBehaviors(WeightedBehaviors) {};

//****************
// BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	SteeringOutput BlendedSteering = {};

	// Calculate the weighted average steering behavior
	for (const auto &WeightedBeh : WeightedBehaviors)
	{
		SteeringOutput IndividualSteering = WeightedBeh.pBehavior->CalculateSteering(DeltaT, Agent);
		BlendedSteering.LinearVelocity += IndividualSteering.LinearVelocity * WeightedBeh.Weight;
		BlendedSteering.AngularVelocity += IndividualSteering.AngularVelocity * WeightedBeh.Weight;
	}

	// Debug drawing: draw the final blended velocity
	if (Agent.GetDebugRenderingEnabled())
	{
		if (UWorld *World = Agent.GetWorld())
		{
			FVector AgentPos = FVector(Agent.GetPosition(), 0.f);
			FVector TargetPos = AgentPos + FVector(BlendedSteering.LinearVelocity, 0.f);
			DrawDebugLine(World, AgentPos, TargetPos, FColor::Yellow, false, -1.f, 0, 2.f);
		}
	}

	return BlendedSteering;
}

float *BlendedSteering::GetWeight(ISteeringBehavior *const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
					  WeightedBehaviors.end(),
					  [SteeringBehavior](const WeightedBehavior &Elem)
					  {
						  return Elem.pBehavior == SteeringBehavior;
					  });

	if (it != WeightedBehaviors.end())
		return &it->Weight;

	return nullptr;
}

//*****************
// PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior *const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	// If non of the behavior return a valid output, last behavior is returned
	return Steering;
}