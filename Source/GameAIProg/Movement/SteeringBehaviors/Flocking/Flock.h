#pragma once

// Toggle this define to compile with spatial partitioning support
#define GAMEAI_USE_SPACE_PARTITIONING

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"
#ifdef GAMEAI_USE_SPACE_PARTITIONING
class CellSpace; // Forward declaration - include in .cpp only
#endif

class Flock final
{
public:
	Flock(
		UWorld *pWorld,
		TSubclassOf<ASteeringAgent> AgentClass,
		int FlockSize = 10,
		float WorldSize = 100.f,
		ASteeringAgent *const pAgentToEvade = nullptr,
		bool bTrimWorld = false);

	~Flock();

	void Tick(float DeltaTime);
	void RenderDebug();
	void ImGuiRender(ImVec2 const &WindowPos, ImVec2 const &WindowSize);

	void RegisterNeighbors(ASteeringAgent *const Agent);
	int GetNrOfNeighbors() const;
	const TArray<ASteeringAgent *> &GetNeighbors() const;

	FVector2D GetAverageNeighborPos() const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const &Target);

private:
	// For debug rendering purposes
	UWorld *pWorld{nullptr};

	int FlockSize{0};
	TArray<ASteeringAgent *> Agents{};

	// Brute-force neighbors (always available)
	TArray<ASteeringAgent *> Neighbors{};

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	// Spatial partitioning members
	std::unique_ptr<CellSpace> pPartitionedSpace;
	int NrOfCellsX{25};
	TArray<FVector2D> OldPositions{};
	bool bUseSpacePartitioning{true}; // Runtime toggle
#endif

	float NeighborhoodRadius{200.f};
	float EvadeRadius{400.f};
	int NrOfNeighbors{0};
	float WorldSize{100.f};
	bool bTrimWorld{false};

	ASteeringAgent *pAgentToEvade{nullptr};

	// Steering Behaviors
	std::unique_ptr<Separation> pSeparationBehavior{};
	std::unique_ptr<Cohesion> pCohesionBehavior{};
	std::unique_ptr<VelocityMatch> pVelMatchBehavior{};
	std::unique_ptr<Seek> pSeekBehavior{};
	std::unique_ptr<Wander> pWanderBehavior{};
	std::unique_ptr<ConditionalEvade> pEvadeBehavior{};

	std::unique_ptr<BlendedSteering> pBlendedSteering{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	// UI and rendering
	bool DebugRenderSteering{false};
	bool DebugRenderNeighborhood{true};
	bool DebugRenderPartitions{true};

	void RenderNeighborhood();
};
