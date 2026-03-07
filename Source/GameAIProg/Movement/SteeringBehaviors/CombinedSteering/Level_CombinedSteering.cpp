#include "Level_CombinedSteering.h"

#include "imgui.h"

// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	// Spawn two agents — both use BlendedSteering (Wander + Evade), evading the mouse
	SpawnAgent(FVector{0, 0, 90});
	SpawnAgent(FVector{300, 300, 90});
}

void ALevel_CombinedSteering::SpawnAgent(const FVector &Location)
{
	AgentData Data;
	Data.WanderBehavior = std::make_unique<Wander>();
	Data.EvadeBehavior = std::make_unique<Evade>();

	// BlendedSteering: Wander (index 0) + Evade (index 1)
	std::vector<BlendedSteering::WeightedBehavior> WeightedBehaviors;
	WeightedBehaviors.push_back({Data.WanderBehavior.get(), 0.5f});
	WeightedBehaviors.push_back({Data.EvadeBehavior.get(), 0.5f});
	Data.Blended = std::make_unique<BlendedSteering>(WeightedBehaviors);

	Data.Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, Location, FRotator::ZeroRotator);
	if (IsValid(Data.Agent))
	{
		Data.Agent->SetSteeringBehavior(Data.Blended.get());
		Data.Agent->SetDebugRenderingEnabled(CanDebugRender);
	}

	Agents.push_back(std::move(Data));
}

void ALevel_CombinedSteering::RemoveAgent(int Index)
{
	if (Index >= 0 && Index < static_cast<int>(Agents.size()))
	{
		if (IsValid(Agents[Index].Agent))
			Agents[Index].Agent->Destroy();
		Agents.erase(Agents.begin() + Index);
	}
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();

	for (auto &Data : Agents)
	{
		Data.Blended.reset();
		Data.WanderBehavior.reset();
		Data.EvadeBehavior.reset();
	}
	Agents.clear();
}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

#pragma region UI
	// UI
	{
		// Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

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
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();

		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
			for (auto &Data : Agents)
			{
				if (IsValid(Data.Agent))
					Data.Agent->SetDebugRenderingEnabled(CanDebugRender);
			}
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
													 TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
													 [this](float InVal)
													 { TrimWorld->SetTrimWorldSize(InVal); });
		}

		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("Agents");
		ImGui::Spacing();

		if (ImGui::Button("Add Agent"))
		{
			SpawnAgent(FVector{FMath::FRandRange(-500.f, 500.f), FMath::FRandRange(-500.f, 500.f), 90});
		}
		ImGui::Spacing();

		for (int i = 0; i < static_cast<int>(Agents.size()); ++i)
		{
			ImGui::PushID(i);
			ImGui::Text("Agent %d", i);
			ImGui::SameLine();
			if (ImGui::Button("x"))
			{
				AgentIndexToRemove = i;
			}
			ImGui::PopID();
		}

		if (AgentIndexToRemove >= 0)
		{
			RemoveAgent(AgentIndexToRemove);
			AgentIndexToRemove = -1;
		}

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// Weight sliders — applied to all agents
		if (Agents.size() > 0 && Agents[0].Blended)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander", Agents[0].Blended->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f, [this](float InVal)
													 {
					for (auto& Data : Agents)
						Data.Blended->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

			ImGuiHelpers::ImGuiSliderFloatWithSetter("Evade", Agents[0].Blended->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f, [this](float InVal)
													 {
					for (auto& Data : Agents)
						Data.Blended->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
		}

		// End
		ImGui::End();
	}
#pragma endregion

	// Combined Steering Update — all agents evade the mouse
	for (auto &Data : Agents)
	{
		if (Data.EvadeBehavior)
		{
			Data.EvadeBehavior->SetTarget(MouseTarget);
		}
	}
}
