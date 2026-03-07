#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"
// SEEK
//*******
//  TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	SteeringOutput Steering{};
	FVector2D Direction = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity = Direction.GetSafeNormal() * Agent.GetMaxLinearSpeed();
	if (Agent.GetDebugRenderingEnabled())
	{
		if (UWorld *World = Agent.GetWorld())
		{
			DrawDebugLine(World, FVector(Agent.GetPosition(), 0.f), FVector(Target.Position, 0.f), FColor::Green, false, -1.f, 0, 1.f);
			DrawDebugSphere(World, FVector(Target.Position, 0.f), 15.f, 8, FColor::Green, false, -1.f, 0, 2.f);
		}
	}

	return Steering;
};

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	SteeringOutput Steering{};
	FVector2D Direction = Agent.GetPosition() - Target.Position;
	Steering.LinearVelocity = Direction.GetSafeNormal() * Agent.GetMaxLinearSpeed();
	if (Agent.GetDebugRenderingEnabled())
	{
		if (UWorld *World = Agent.GetWorld())
		{
			DrawDebugLine(World, FVector(Agent.GetPosition(), 0.f), FVector(Agent.GetPosition() + Steering.LinearVelocity, 0.f), FColor::Red, false, -1.f, 0, 1.f);
			DrawDebugSphere(World, FVector(Target.Position, 0.f), 15.f, 8, FColor::Red, false, -1.f, 0, 2.f);
		}
	}

	return Steering;
};

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	SteeringOutput Steering{};
	if (m_OriginalMaxSpeed < 0.f)
	{
		m_OriginalMaxSpeed = Agent.GetMaxLinearSpeed();
	}
	// Parameters
	const float SlowRadius = 800.f;
	const float TargetRadius = 50.f;

	if (Agent.GetDebugRenderingEnabled())
	{
		if (UWorld *World = Agent.GetWorld())
		{
			DrawDebugSphere(World, FVector(Target.Position, 0.f), SlowRadius, 16, FColor::Green, false, -1.f, 0, 2.f);
			DrawDebugSphere(World, FVector(Target.Position, 0.f), TargetRadius, 16, FColor::Red, false, -1.f, 0, 2.f);
		}
	}

	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Size();
	// Default to the original full speed
	float DesiredSpeed = m_OriginalMaxSpeed;
	// Stop if inside TargetRadius
	if (Distance < TargetRadius)
	{
		DesiredSpeed = 0.f;
		// We can early exit, but we should set the agent speed to 0 first to be consistent
		Agent.SetMaxLinearSpeed(DesiredSpeed);
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}
	// "Gradually decrease speed"
	if (Distance < SlowRadius)
	{
		DesiredSpeed = m_OriginalMaxSpeed * (Distance / SlowRadius);
	}
	Agent.SetMaxLinearSpeed(DesiredSpeed);
	Steering.LinearVelocity = ToTarget.GetSafeNormal() * Agent.GetMaxLinearSpeed();
	return Steering;
}

void Arrive::SetTargetRadius(float X) {

};

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	SteeringOutput Steering{};
	FVector2D Direction = Target.Position - Agent.GetPosition();
	// Safety check
	if (Direction.IsNearlyZero())
		return Steering;

	if (Agent.GetDebugRenderingEnabled())
	{
		if (UWorld *World = Agent.GetWorld())
		{
			DrawDebugLine(World, FVector(Agent.GetPosition(), 0), FVector(Target.Position, 0), FColor::Yellow, false, -1, 0, 2.0f);
			float CurrentRad = FMath::DegreesToRadians(Agent.GetRotation());
			FVector FacingDir = FVector(FMath::Cos(CurrentRad), FMath::Sin(CurrentRad), 0);
			DrawDebugLine(World, FVector(Agent.GetPosition(), 0), FVector(Agent.GetPosition(), 0) + (FacingDir * 100.f), FColor::Red, false, -1, 0, 2.0f);
		}
	}

	// [IMPORTANT] Atan2 returns Radians. We MUST convert to Degrees.
	float TargetAngle = FMath::RadiansToDegrees(FMath::Atan2(Direction.Y, Direction.X));
	float CurrentAngle = Agent.GetRotation();
	float AngleDiff = TargetAngle - CurrentAngle;
	while (AngleDiff > 180.f)
		AngleDiff -= 360.f;
	while (AngleDiff < -180.f)
		AngleDiff += 360.f;
	float MaxSpeed = Agent.GetMaxAngularSpeed();
	if (FMath::Abs(AngleDiff) < 2.0f) // 2 Degree Deadzone
	{
		Steering.AngularVelocity = 0.f;
	}
	else
	{
		Steering.AngularVelocity = (AngleDiff > 0) ? MaxSpeed : -MaxSpeed;

		float SlowAngle = 30.f; // Start slowing down at 30 degrees difference
		if (FMath::Abs(AngleDiff) < SlowAngle)
			Steering.AngularVelocity *= (FMath::Abs(AngleDiff) / SlowAngle);
	}

	// Face behavior strictly uses Angular Velocity, Linear must be zero
	Steering.LinearVelocity = FVector2D::ZeroVector;

	return Steering;
};

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	SteeringOutput Steering{};

	FVector2D PredictedPos = GetPredictedPosition(Agent);
	FVector2D Direction = PredictedPos - Agent.GetPosition();
	Steering.LinearVelocity = Direction.GetSafeNormal() * Agent.GetMaxLinearSpeed();

	if (Agent.GetDebugRenderingEnabled())
	{
		if (UWorld *World = Agent.GetWorld())
		{
			DrawDebugLine(World, FVector(Agent.GetPosition(), 0.f), FVector(PredictedPos, 0.f), FColor::Orange, false, -1.f, 0, 1.f);
			DrawDebugSphere(World, FVector(PredictedPos, 0.f), 20.f, 8, FColor::Orange, false, -1.f, 0, 2.f);
			DrawDebugPoint(World, FVector(Target.Position, 0.f), 10.f, FColor::White, false, -1.f);
		}
	}
	return Steering;
};

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	SteeringOutput Steering{};

	FVector2D PredictedPos = GetPredictedPosition(Agent);
	FVector2D Direction = Agent.GetPosition() - PredictedPos;
	Steering.LinearVelocity = Direction.GetSafeNormal() * Agent.GetMaxLinearSpeed();

	if (Agent.GetDebugRenderingEnabled())
	{
		if (UWorld *World = Agent.GetWorld())
		{
			DrawDebugLine(World, FVector(Agent.GetPosition(), 0.f), FVector(Agent.GetPosition() + Steering.LinearVelocity, 0.f), FColor::Magenta, false, -1.f, 0, 1.f);
			DrawDebugSphere(World, FVector(PredictedPos, 0.f), 20.f, 8, FColor::Magenta, false, -1.f, 0, 2.f);
		}
	}
	return Steering;
};

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent &Agent)
{
	m_WanderAngle += FMath::FRandRange(-1.f, 1.f) * m_MaxAngleChange;
	// Clamp to prevent the target drifting too far to one side (causing circular motion)
	m_WanderAngle = FMath::Clamp(m_WanderAngle, -m_MaxWanderAngle, m_MaxWanderAngle);

	float CurrentRad = FMath::DegreesToRadians(Agent.GetRotation());
	FVector2D Heading{FMath::Cos(CurrentRad), FMath::Sin(CurrentRad)};
	FVector2D CircleCenter = Agent.GetPosition() + (Heading * m_OffsetDistance);

	float TotalAngle = m_WanderAngle + CurrentRad;
	FVector2D OffsetPoint{FMath::Cos(TotalAngle) * m_Radius, FMath::Sin(TotalAngle) * m_Radius};

	FVector2D NewTarget = CircleCenter + OffsetPoint;
	this->SetTarget(FTargetData(NewTarget));

	if (Agent.GetDebugRenderingEnabled())
	{
		if (UWorld *World = Agent.GetWorld())
		{
			FVector Center3D = FVector(CircleCenter, 0.f);
			FVector Target3D = FVector(NewTarget, 0.f);
			DrawDebugCircle(World, Center3D, m_Radius, 12, FColor::Cyan, false, -1.f, 0, 2.f, FVector(1, 0, 0), FVector(0, 1, 0));
			DrawDebugLine(World, FVector(Agent.GetPosition(), 0.f), Center3D, FColor::Cyan, false, -1.f, 0, 1.f);
			DrawDebugSphere(World, Target3D, 15.f, 8, FColor::Red, false, -1.f, 0, 2.f);
		}
	}
	return Seek::CalculateSteering(DeltaT, Agent);
};

// HELPERS
FVector2D ISteeringBehavior::GetPredictedPosition(const ASteeringAgent &Agent) const
{
	float Distance = FVector2D::Distance(Target.Position, Agent.GetPosition());
	float AgentSpeed = Agent.GetMaxLinearSpeed();
	// Prevent division by zero
	if (AgentSpeed <= 0.f)
		return Target.Position;
	// Time = Distance / Speed
	float PredictionTime = Distance / AgentSpeed;
	constexpr float MaxPredictionTime = 2.0f; // 2sec !sure about it ASK adding more to evade ?
	if (PredictionTime > MaxPredictionTime)
	{
		PredictionTime = MaxPredictionTime;
	}
	// FuturePos = CurrentPos + (Velocity * Time)
	return Target.Position + (Target.LinearVelocity * PredictionTime);
};
