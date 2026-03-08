#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "DrawDebugHelpers.h"
#ifdef GAMEAI_USE_SPACE_PARTITIONING
#include "../SpacePartitioning/SpacePartitioning.h"
#endif

Flock::Flock(
	UWorld *pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent *const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}, FlockSize{FlockSize}, pAgentToEvade{pAgentToEvade}, WorldSize{WorldSize}, bTrimWorld{bTrimWorld}
{
	Agents.SetNum(FlockSize);

	// Initialize steering behaviors
	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<ConditionalEvade>();
	pEvadeBehavior->SetEvadeRadius(EvadeRadius);

	// BlendedSteering: Cohesion + Separation + VelocityMatch + Seek + Wander
	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>{
			{pCohesionBehavior.get(), 2.f},
			{pSeparationBehavior.get(), 10.f},
			{pVelMatchBehavior.get(), 2.f},
			{pSeekBehavior.get(), 1.f},
			{pWanderBehavior.get(), 1.f}});

	// PrioritySteering: ConditionalEvade > BlendedSteering
	pPrioritySteering = std::make_unique<PrioritySteering>(
		std::vector<ISteeringBehavior *>{
			pEvadeBehavior.get(),
			pBlendedSteering.get()});

	// Memory Pool: pre-allocate neighbor array to FlockSize (no push_back/clear)
	Neighbors.SetNum(FlockSize);
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, WorldSize, WorldSize, NrOfCellsX, NrOfCellsX, FlockSize);
	OldPositions.SetNum(FlockSize);
#endif
	NrOfNeighbors = 0;

	// Spawn agents
	for (int i = 0; i < FlockSize; ++i)
	{
		FVector SpawnLocation = FVector(
			FMath::FRandRange(-WorldSize * 0.5f, WorldSize * 0.5f),
			FMath::FRandRange(-WorldSize * 0.5f, WorldSize * 0.5f),
			90.f);

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnLocation, FRotator::ZeroRotator);
		if (IsValid(Agents[i]))
		{
			Agents[i]->SetSteeringBehavior(pPrioritySteering.get());
			Agents[i]->SetDebugRenderingEnabled(false);
			Agents[i]->SetActorTickEnabled(false);
#ifdef GAMEAI_USE_SPACE_PARTITIONING
			pPartitionedSpace->AddAgent(*Agents[i]);
			OldPositions[i] = Agents[i]->GetPosition();
#endif
		}
	}
}

Flock::~Flock()
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	pPartitionedSpace.reset();
#endif
	// Steering behaviors are cleaned up by unique_ptrs
	// Agents are UE actors managed by GC
}

void Flock::Tick(float DeltaTime)
{
	// Update evade target
	if (IsValid(pAgentToEvade))
	{
		FTargetData EvadeTarget{
			pAgentToEvade->GetPosition(),
			pAgentToEvade->GetRotation(),
			pAgentToEvade->GetLinearVelocity(),
			pAgentToEvade->GetAngularVelocity()};
		pEvadeBehavior->SetTarget(EvadeTarget);
	}

	for (int i = 0; i < FlockSize; ++i)
	{
		if (!IsValid(Agents[i]))
			continue;

#ifdef GAMEAI_USE_SPACE_PARTITIONING
		OldPositions[i] = Agents[i]->GetPosition();
#endif

		RegisterNeighbors(Agents[i]);
		Agents[i]->Tick(DeltaTime);

#ifdef GAMEAI_USE_SPACE_PARTITIONING
		pPartitionedSpace->UpdateAgentCell(*Agents[i], OldPositions[i]);
#endif
	}
}

void Flock::RenderDebug()
{
	// Only enable debug rendering on the first agent
	for (int i = 0; i < FlockSize; ++i)
	{
		if (IsValid(Agents[i]))
		{
			Agents[i]->SetDebugRenderingEnabled(DebugRenderSteering && i == 0);
		}
	}

	if (DebugRenderNeighborhood)
	{
		RenderNeighborhood();
	}

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (DebugRenderPartitions)
	{
		pPartitionedSpace->RenderCells();
	}
#endif
}

void Flock::ImGuiRender(ImVec2 const &WindowPos, ImVec2 const &WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	// UI
	{
		// Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		// Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		// Debug rendering checkboxes
		ImGui::Checkbox("Debug Steering", &DebugRenderSteering);
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
#ifdef GAMEAI_USE_SPACE_PARTITIONING
		{
			bool bNotUsingPartitioning = !bUseSpacePartitioning;
			if (ImGui::Checkbox("Use Space Partitioning", &bNotUsingPartitioning))
				bUseSpacePartitioning = !bNotUsingPartitioning;
			ImGui::Checkbox("Debug Partitions", &DebugRenderPartitions);
		}
#endif

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// Sliders for steering behavior weights
		if (pBlendedSteering)
		{
			auto &Behaviors = pBlendedSteering->GetWeightedBehaviorsRef();
			if (Behaviors.size() > 0)
				ImGui::SliderFloat("Cohesion", &Behaviors[0].Weight, 0.f, 20.f, "%.1f");
			if (Behaviors.size() > 1)
				ImGui::SliderFloat("Separation", &Behaviors[1].Weight, 0.f, 20.f, "%.1f");
			if (Behaviors.size() > 2)
				ImGui::SliderFloat("VelocityMatch", &Behaviors[2].Weight, 0.f, 20.f, "%.1f");
			if (Behaviors.size() > 3)
				ImGui::SliderFloat("Seek", &Behaviors[3].Weight, 0.f, 20.f, "%.1f");
			if (Behaviors.size() > 4)
				ImGui::SliderFloat("Wander", &Behaviors[4].Weight, 0.f, 20.f, "%.1f");
		}
		// End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	if (Agents.Num() == 0 || !IsValid(Agents[0]))
		return;

	// Debug render neighborhood for the first agent
	RegisterNeighbors(Agents[0]);
	FVector AgentPos = FVector(Agents[0]->GetPosition(), 0.f);

	DrawDebugCircle(pWorld, AgentPos, NeighborhoodRadius, 48, FColor::Cyan, false, -1.f, 0, 3.f, FVector(1, 0, 0), FVector(0, 1, 0));

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (bUseSpacePartitioning)
	{
		// Draw the query bounding box used by spatial partitioning
		FVector2D Pos2D = Agents[0]->GetPosition();
		FVector BBoxMin(Pos2D.X - NeighborhoodRadius, Pos2D.Y - NeighborhoodRadius, 0.f);
		FVector BBoxMax(Pos2D.X + NeighborhoodRadius, Pos2D.Y + NeighborhoodRadius, 0.f);
		FVector Center = (BBoxMin + BBoxMax) * 0.5f;
		FVector Extent = (BBoxMax - BBoxMin) * 0.5f;
		DrawDebugBox(pWorld, Center, Extent, FColor::Green, false, -1.f, 0, 2.f);
	}
#endif

	// Highlight the agent being inspected
	DrawDebugSphere(pWorld, AgentPos, 30.f, 8, FColor::Magenta, false, -1.f, 0, 2.f);

	int NeighborCount = GetNrOfNeighbors();
	const TArray<ASteeringAgent *> &NeighborArray = GetNeighbors();
	for (int i = 0; i < NeighborCount; ++i)
	{
		if (IsValid(NeighborArray[i]))
		{
			FVector NeighborPos = FVector(NeighborArray[i]->GetPosition(), 0.f);
			DrawDebugLine(pWorld, AgentPos, NeighborPos, FColor::Yellow, false, -1.f, 0, 2.5f);
			DrawDebugPoint(pWorld, NeighborPos, 10.f, FColor::Orange, false, -1.f, 0);
		}
	}
}

void Flock::RegisterNeighbors(ASteeringAgent *const pAgent)
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (bUseSpacePartitioning)
	{
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
		return;
	}
#endif
	// Brute-force: reset count, overwrite existing slots (no push_back/clear)
	NrOfNeighbors = 0;

	for (int i = 0; i < FlockSize; ++i)
	{
		if (!IsValid(Agents[i]) || Agents[i] == pAgent)
			continue;

		float DistSq = FVector2D::DistSquared(pAgent->GetPosition(), Agents[i]->GetPosition());
		if (DistSq < NeighborhoodRadius * NeighborhoodRadius)
		{
			Neighbors[NrOfNeighbors] = Agents[i];
			++NrOfNeighbors;
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (bUseSpacePartitioning)
		return pPartitionedSpace->GetNrOfNeighbors();
#endif
	return NrOfNeighbors;
}

const TArray<ASteeringAgent *> &Flock::GetNeighbors() const
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (bUseSpacePartitioning)
		return pPartitionedSpace->GetNeighbors();
#endif
	return Neighbors;
}

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

	int neighborCount = GetNrOfNeighbors();
	if (neighborCount == 0)
		return avgPosition;

	const TArray<ASteeringAgent *> &neighbors = GetNeighbors();
	for (int i = 0; i < neighborCount; ++i)
	{
		avgPosition += neighbors[i]->GetPosition();
	}

	avgPosition /= static_cast<float>(neighborCount);
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

	int neighborCount = GetNrOfNeighbors();
	if (neighborCount == 0)
		return avgVelocity;

	const TArray<ASteeringAgent *> &neighbors = GetNeighbors();
	for (int i = 0; i < neighborCount; ++i)
	{
		avgVelocity += neighbors[i]->GetLinearVelocity();
	}

	avgVelocity /= static_cast<float>(neighborCount);
	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const &Target)
{
	pSeekBehavior->SetTarget(Target);
}
