#include "NavGraphPathfinding.h"

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

	// We have valid start/end triangles and they are not the same
	if (!startTriangle || !endTriangle)
		return finalPath;

	if (*startTriangle == *endTriangle)
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

	// Connect nodes on edges of endTriangle to end node
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

	// Run A star on new graph
	AStar astar(pClonedGraph.get(), HeuristicFunctions::Euclidean);
	auto nodes = astar.FindPath(
		pClonedGraph->GetNode(startNodeId).get(),
		pClonedGraph->GetNode(endNodeId).get());

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