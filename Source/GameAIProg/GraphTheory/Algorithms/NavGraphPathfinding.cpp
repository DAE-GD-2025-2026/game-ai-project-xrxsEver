#include "NavGraphPathfinding.h"

#include <limits>

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D &startPos, const FVector2D &endPos,
													NavGraph *const pNavGraph, std::vector<FVector2D> &debugNodePositions, std::vector<NavLine> &debugPortals)
{
	// Create the path to return
	std::vector<FVector2D> finalPath{};

	// Get the start and endTriangle
	auto const *pNavPoly = pNavGraph->GetNavPolygon();
	auto const *startTriangle = pNavPoly->GetTriangleAtPosition(startPos, true);
	auto const *endTriangle = pNavPoly->GetTriangleAtPosition(endPos, true);

	// The agent itself must be on the navmesh, otherwise we have nothing to path from.
	if (!startTriangle)
		return finalPath;

	// Note: a null endTriangle means the goal was clicked outside the navmesh. We do NOT
	// bail here - instead we let A* fail to reach the (unconnected) end node so the
	// Fallback Path logic below can route the agent to the closest reachable node.
	if (endTriangle && *startTriangle == *endTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	//=> Start looking for a path
	// Copy the graph
	auto pClonedGraph = pNavGraph->Clone();

	// Create Extra node for the Start Node (Agent's position)
	int startNodeId = pClonedGraph->AddNode(std::make_unique<NavGraphNode>(startPos, -1));

	// Connect start node to nodes on edges of startTriangle
	auto startEdges = startTriangle->GetEdges();
	for (auto const &edge : startEdges)
	{
		int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
		if (edgeIdx >= 0)
		{
			int nodeId = pClonedGraph->GetNodeIdFromEdgeIndex(edgeIdx);
			if (nodeId != Graphs::InvalidNodeId)
			{
				auto connection = std::make_unique<Connection>(startNodeId, nodeId);
				connection->SetWeight(FVector2D::Distance(startPos, pClonedGraph->GetNode(nodeId)->GetPosition()));
				pClonedGraph->AddConnection(std::move(connection));
			}
		}
	}

	// Create extra node for the endNode
	int endNodeId = pClonedGraph->AddNode(std::make_unique<NavGraphNode>(endPos, -1));

	// Connect nodes on edges of endTriangle to end node.
	// If endTriangle is null (goal clicked off the navmesh) the end node stays unconnected,
	// which makes A* fail to reach it and triggers the Fallback Path below.
	if (endTriangle)
	{
		auto endEdges = endTriangle->GetEdges();
		for (auto const &edge : endEdges)
		{
			int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
			if (edgeIdx >= 0)
			{
				int nodeId = pClonedGraph->GetNodeIdFromEdgeIndex(edgeIdx);
				if (nodeId != Graphs::InvalidNodeId)
				{
					auto connection = std::make_unique<Connection>(nodeId, endNodeId);
					connection->SetWeight(FVector2D::Distance(endPos, pClonedGraph->GetNode(nodeId)->GetPosition()));
					pClonedGraph->AddConnection(std::move(connection));
				}
			}
		}
	}

	// Run A star on new graph
	AStar astar(pClonedGraph.get(), HeuristicFunctions::Euclidean);
	auto nodes = astar.FindPath(
		pClonedGraph->GetNode(startNodeId).get(),
		pClonedGraph->GetNode(endNodeId).get());

	// Fallback Path: if the goal is unreachable (A* never expanded the end node, e.g. the
	// start and goal lie in disconnected parts of the navmesh), pathfind to the graph node
	// closest to the original goal instead so the agent still moves as near as possible.
	bool const bReachedGoal = !nodes.empty() && nodes.back()->GetId() == endNodeId;
	if (!bReachedGoal)
	{
		Node *pClosestNode = nullptr;
		float closestDistSq = std::numeric_limits<float>::max();
		for (Node *pNode : pClonedGraph->GetActiveNodes())
		{
			// Skip the helper start/end nodes we injected above
			if (pNode->GetId() == startNodeId || pNode->GetId() == endNodeId)
				continue;

			float const distSq = FVector2D::DistSquared(pNode->GetPosition(), endPos);
			if (distSq < closestDistSq)
			{
				closestDistSq = distSq;
				pClosestNode = pNode;
			}
		}

		if (pClosestNode)
		{
			nodes = astar.FindPath(
				pClonedGraph->GetNode(startNodeId).get(),
				pClonedGraph->GetNode(pClosestNode->GetId()).get());
		}
	}

	// Debug Visualisation
	for (auto const *node : nodes)
	{
		debugNodePositions.push_back(node->GetPosition());
		finalPath.push_back(node->GetPosition());
	}

	// Extra: Run optimiser on new graph
	debugPortals = SSFA::FindPortals(nodes, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());

	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D &startPos, const FVector2D &endPos, NavGraph *const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}