#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> &&NavPoly)
	: Graph{false}, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph &Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const &OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode *>(OtherNode.get())));
	}

	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const &OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const &pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode *>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}

	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	// 1. Go over all the edges of the navigation mesh and create nodes
	auto const &edges = pNavPoly->GetEdges();
	auto const &triangles = pNavPoly->GetTriangles();

	for (int edgeIdx = 0; edgeIdx < static_cast<int>(edges.size()); ++edgeIdx)
	{
		// Check if that line is connected to another triangle
		int triangleCount = 0;
		for (auto const &tri : triangles)
		{
			if (tri.HasEdge(edges[edgeIdx]))
			{
				++triangleCount;
			}
		}

		if (triangleCount >= 2)
		{
			// Create node at the middle of the line
			FVector2D p1{edges[edgeIdx].GetP1(*pNavPoly)};
			FVector2D p2{edges[edgeIdx].GetP2(*pNavPoly)};
			FVector2D midPoint = (p1 + p2) * 0.5f;

			AddNode(std::make_unique<NavGraphNode>(midPoint, edgeIdx));
		}
	}

	// 2. Create connections now that every node is created
	for (auto const &tri : triangles)
	{
		std::vector<int> nodeIds;
		auto triEdges = tri.GetEdges();

		for (auto const &edge : triEdges)
		{
			int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
			if (edgeIdx >= 0)
			{
				int nodeId = GetNodeIdFromEdgeIndex(edgeIdx);
				if (nodeId != Graphs::InvalidNodeId)
				{
					nodeIds.push_back(nodeId);
				}
			}
		}

		// 2 valid nodes -> 1 connection
		if (nodeIds.size() == 2)
		{
			AddConnection(std::make_unique<Connection>(nodeIds[0], nodeIds[1]));
		}
		// 3 valid nodes -> 3 connections
		else if (nodeIds.size() == 3)
		{
			AddConnection(std::make_unique<Connection>(nodeIds[0], nodeIds[1]));
			AddConnection(std::make_unique<Connection>(nodeIds[1], nodeIds[2]));
			AddConnection(std::make_unique<Connection>(nodeIds[0], nodeIds[2]));
		}
	}

	// 3. Set the connections cost to the actual distance
	SetConnectionCostsToDistances();
}
