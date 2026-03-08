#include "SpacePartitioning.h"
#include "DrawDebugHelpers.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = {Left, Bottom};
	BoundingBox.Max = {BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height};
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
		{
			{left, bottom},
			{left, bottom + height},
			{left + width, bottom + height},
			{left + width, bottom},
		};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld *pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}, SpaceWidth{Width}, SpaceHeight{Height}, NrOfRows{Rows}, NrOfCols{Cols}, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);

	// calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	// Create cells in row-major order
	for (int row = 0; row < NrOfRows; ++row)
	{
		for (int col = 0; col < NrOfCols; ++col)
		{
			float left = -Width * 0.5f + col * CellWidth;
			float bottom = -Height * 0.5f + row * CellHeight;
			Cells.push_back(Cell(left, bottom, CellWidth, CellHeight));
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent &Agent)
{
	int idx = PositionToIndex(Agent.GetPosition());
	Cells[idx].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent &Agent, const FVector2D &OldPos)
{
	int oldIdx = PositionToIndex(OldPos);
	int newIdx = PositionToIndex(Agent.GetPosition());

	if (oldIdx != newIdx)
	{
		Cells[oldIdx].Agents.remove(&Agent);
		Cells[newIdx].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent &Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	FVector2D AgentPos = Agent.GetPosition();

	// Build query rect around the agent
	FRect QueryRect;
	QueryRect.Min = {AgentPos.X - QueryRadius, AgentPos.Y - QueryRadius};
	QueryRect.Max = {AgentPos.X + QueryRadius, AgentPos.Y + QueryRadius};

	for (const Cell &cell : Cells)
	{
		if (!DoRectsOverlap(QueryRect, cell.BoundingBox))
			continue;

		for (ASteeringAgent *pOther : cell.Agents)
		{
			if (pOther == &Agent)
				continue;

			float DistSq = FVector2D::DistSquared(AgentPos, pOther->GetPosition());
			if (DistSq < QueryRadius * QueryRadius)
			{
				Neighbors[NrOfNeighbors] = pOther;
				++NrOfNeighbors;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell &c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	for (const Cell &cell : Cells)
	{
		std::vector<FVector2D> points = cell.GetRectPoints();
		for (int i = 0; i < 4; ++i)
		{
			FVector start(points[i], 90.f);
			FVector end(points[(i + 1) % 4], 90.f);
			DrawDebugLine(pWorld, start, end, FColor::Cyan, false, -1.f, 0, 1.f);
		}
	}
}

int CellSpace::PositionToIndex(FVector2D const &Pos) const
{
	int col = static_cast<int>((Pos.X + SpaceWidth * 0.5f) / CellWidth);
	int row = static_cast<int>((Pos.Y + SpaceHeight * 0.5f) / CellHeight);
	col = FMath::Clamp(col, 0, NrOfCols - 1);
	row = FMath::Clamp(row, 0, NrOfRows - 1);
	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const &RectA, FRect const &RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X)
		return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y)
		return false;

	// If they are not separated, they must overlap
	return true;
}