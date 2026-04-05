#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
	{
	public:
		//=== SSFA Functions ===
		//--- References ---
		// http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
		// https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
		static std::vector<NavLine> FindPortals(std::vector<Node *> const &Path, TriPolygon const &NavPoly)
		{
			// Container
			std::vector<NavLine> Portals = {};

			// For each node received, get it's corresponding line
			for (size_t i = 0; i < Path.size(); ++i)
			{
				NavGraphNode *pNavNode = reinterpret_cast<NavGraphNode *>(Path[i]);
				int edgeIdx = pNavNode->GetEdgeIdx();

				if (edgeIdx < 0)
				{
					// Degenerate portal for start/end nodes
					Portals.push_back(NavLine{Path[i]->GetPosition(), Path[i]->GetPosition()});
					continue;
				}

				auto const &edges = NavPoly.GetEdges();
				auto const &edge = edges[edgeIdx];

				FVector2D p1{edge.GetP1(NavPoly)};
				FVector2D p2{edge.GetP2(NavPoly)};

				// Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point
				FVector2D direction;
				if (i + 1 < Path.size())
					direction = Path[i + 1]->GetPosition() - Path[i > 0 ? i - 1 : i]->GetPosition();
				else
					direction = Path[i]->GetPosition() - Path[i - 1]->GetPosition();

				FVector2D toP1 = p1 - Path[i]->GetPosition();
				float cross = FVector2D::CrossProduct(direction, toP1);

				if (cross > 0.f)
				{
					std::swap(p1, p2);
				}

				// Store portal
				Portals.push_back(NavLine{p1, p2});
			}

			// Add degenerate portal to force end evaluation
			FVector2D endPos = Path.back()->GetPosition();
			Portals.push_back(NavLine{endPos, endPos});

			return Portals;
		}

		static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const &Portals, TriPolygon const &NavPoly)
		{
			std::vector<FVector2D> Path{};
			if (Portals.empty())
				return Path;

			// P1 == right point of portal, P2 == left point of portal
			FVector2D apexPos = Portals[0].P1;
			int apexIndex = 0;
			int rightLegIndex = 0;
			int leftLegIndex = 0;

			FVector2D rightLeg = Portals[0].P1 - apexPos;
			FVector2D leftLeg = Portals[0].P2 - apexPos;

			// Add the apexPoint to the path (first path point)
			Path.push_back(apexPos);

			// Loop over all the portals (Starting from the second portal)
			for (int i = 1; i < static_cast<int>(Portals.size()); ++i)
			{
				NavLine const &portal = Portals[i];

				//--- RIGHT CHECK ---
				// 1. See if moving funnel inwards - RIGHT
				FVector2D newRightLeg = portal.P1 - apexPos;

				if (FVector2D::CrossProduct(rightLeg, newRightLeg) >= 0.f)
				{
					// 2. See if new line degenerates a line segment - RIGHT
					if (FVector2D::CrossProduct(leftLeg, newRightLeg) <= 0.f)
					{
						// Not crossing left leg - tighten
						rightLeg = newRightLeg;
						rightLegIndex = i;
					}
					else
					{
						// Leftleg becomes new apex point
						apexPos = apexPos + leftLeg;
						apexIndex = leftLegIndex;

						// Calculate new legs (if not the end)
						int portalIdx = leftLegIndex + 1;
						leftLegIndex = portalIdx;
						rightLegIndex = portalIdx;

						Path.push_back(apexPos);

						if (portalIdx < static_cast<int>(Portals.size()))
						{
							rightLeg = Portals[portalIdx].P1 - apexPos;
							leftLeg = Portals[portalIdx].P2 - apexPos;
						}

						i = portalIdx - 1;
						continue;
					}
				}

				//--- LEFT CHECK ---
				// 1. See if moving funnel inwards - LEFT
				FVector2D newLeftLeg = portal.P2 - apexPos;

				if (FVector2D::CrossProduct(leftLeg, newLeftLeg) <= 0.f)
				{
					// 2. See if new line degenerates a line segment - LEFT
					if (FVector2D::CrossProduct(rightLeg, newLeftLeg) >= 0.f)
					{
						// Not crossing right leg - tighten
						leftLeg = newLeftLeg;
						leftLegIndex = i;
					}
					else
					{
						// Rightleg becomes new apex point
						apexPos = apexPos + rightLeg;
						apexIndex = rightLegIndex;

						// Calculate new legs (if not the end)
						int portalIdx = rightLegIndex + 1;
						leftLegIndex = portalIdx;
						rightLegIndex = portalIdx;

						Path.push_back(apexPos);

						if (portalIdx < static_cast<int>(Portals.size()))
						{
							rightLeg = Portals[portalIdx].P1 - apexPos;
							leftLeg = Portals[portalIdx].P2 - apexPos;
						}

						i = portalIdx - 1;
						continue;
					}
				}
			}

			// Add last path point
			Path.push_back(Portals.back().P1);

			return Path;
		}

	private:
		SSFA() {};
		~SSFA() {};
	};
}
