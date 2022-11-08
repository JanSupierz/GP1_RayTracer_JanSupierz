#pragma once
#include <cassert>

#include "Math.h"
#include "vector"

namespace dae
{
#pragma region GEOMETRY
		struct Sphere
		{
			Vector3 origin{};
			float radius{};

			unsigned char materialIndex{ 0 };
		};

		struct Plane
		{
			Vector3 origin{};
			Vector3 normal{};

			unsigned char materialIndex{ 0 };
		};

		enum class TriangleCullMode
		{
			FrontFaceCulling,
			BackFaceCulling,
			NoCulling
		};

		struct Triangle
		{
			Triangle() = default;
			Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal) :
				v0{ _v0 }, v1{ _v1 }, v2{ _v2 }, normal{ _normal.Normalized() }
			{
			}

			Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
				v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
			{
				const Vector3 edgeV0V1 = v1 - v0;
				const Vector3 edgeV0V2 = v2 - v0;
				normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
			}

			Vector3 v0{};
			Vector3 v1{};
			Vector3 v2{};

			Vector3 normal{};

			TriangleCullMode cullMode{};
			unsigned char materialIndex{};
		};

		struct BVHNode
		{
			Vector3 minAABB, maxAABB;
			uint32_t leftFirst, nrPrimitives;
		};

		struct Aabb
		{
			Vector3 min{ INFINITY,INFINITY,INFINITY }, max{ -INFINITY,-INFINITY,-INFINITY };

			void grow(Vector3 pos) 
			{ 
				min = Vector3::Min(min, pos);
				max = Vector3::Max(max, pos);
			}

			void grow(Sphere* sphere)
			{
				const Vector3 origin = sphere->origin;
				const Vector3 radius = sphere->radius * Vector3{ 1.f,1.f,1.f };

				min = Vector3::Min(origin - radius, min);
				max = Vector3::Max(origin + radius, max);
			}

			float area()
			{
				Vector3 extent = max - min; // box extent
				return extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
			}
		};


		struct TriangleMesh
		{
			TriangleMesh() = default;

			TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
				positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
			{
			}

			std::vector<Vector3> positions{};
			std::vector<Vector3> normals{};
			std::vector<int> indices{};

			unsigned char materialIndex{};

			uint32_t nrTriangles{};

			TriangleCullMode cullMode{ TriangleCullMode::BackFaceCulling };

			Matrix rotationTransform{};
			Matrix translationTransform{};
			Matrix scaleTransform{};

			std::vector<Vector3> transformedPositions{};
			std::vector<Vector3> transformedNormals{};

			std::vector<BVHNode> bvhNodes{};
			uint32_t rootNodeIndex{};
			uint32_t numberUsedNodes{};

			void Translate(const Vector3& translation)
			{
				translationTransform = Matrix::CreateTranslation(translation);
			}

			void RotateY(float yaw)
			{
				rotationTransform = Matrix::CreateRotationY(yaw);
			}

			void Scale(const Vector3& scale)
			{
				scaleTransform = Matrix::CreateScale(scale);
			}

			void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
			{
				int startIndex = static_cast<int>(positions.size());

				positions.push_back(triangle.v0);
				positions.push_back(triangle.v1);
				positions.push_back(triangle.v2);

				indices.push_back(startIndex);
				indices.push_back(++startIndex);
				indices.push_back(++startIndex);

				normals.push_back(triangle.normal);

				//Not ideal, but making sure all vertices are updated
				if (!ignoreTransformUpdate)
					UpdateTransforms();
			}

			void UpdateTransforms()
			{
				transformedNormals.clear();
				transformedPositions.clear();
				transformedNormals.reserve(normals.size());
				transformedPositions.reserve(positions.size());

				auto transformMatrix = scaleTransform * rotationTransform * translationTransform;

				for (size_t i = 0; i < positions.size(); i++)
				{
					transformedPositions.emplace_back(transformMatrix.TransformPoint(positions[i]));
				}

				for (size_t i = 0; i < normals.size(); i++)
				{
					transformedNormals.emplace_back(transformMatrix.TransformVector(normals[i]).Normalized());
				}

				RefitBVH();
			}

			void RefitBVH()
			{
				for (int index = numberUsedNodes - 1; index >= 0; index--)
				{
					BVHNode& node{ bvhNodes[index] };

					if (node.nrPrimitives != 0)
					{
						UpdateAABB(index);
						continue;
					}

					BVHNode& leftChild{ bvhNodes[node.leftFirst] };
					BVHNode& rightChild{bvhNodes[node.leftFirst + 1]};

					node.minAABB = Vector3::Min(leftChild.minAABB, rightChild.minAABB);
					node.maxAABB = Vector3::Max(leftChild.maxAABB, rightChild.maxAABB);
				}
			}

			void UpdateAABB(uint32_t nodeIndex)
			{
				BVHNode& node{ bvhNodes[nodeIndex] };

				uint32_t start{ node.leftFirst * 3 };
				uint32_t end{ start + node.nrPrimitives * 3 };

				node.minAABB = Vector3{ INFINITY,INFINITY,INFINITY };
				node.maxAABB = Vector3{ -INFINITY,-INFINITY,-INFINITY };

				for (uint32_t index{ start }; index < end; ++index)
				{
					node.minAABB = Vector3::Min(transformedPositions[indices[index]], node.minAABB);
					node.maxAABB = Vector3::Max(transformedPositions[indices[index]], node.maxAABB);
				}
			}

			void InitBVH()
			{
				nrTriangles = static_cast<int>(indices.size()) / 3;

				bvhNodes.reserve(nrTriangles * 2 - 1);

				for (size_t index = 0; index < bvhNodes.capacity(); index++)
				{
					bvhNodes.push_back(BVHNode{});
				}

				rootNodeIndex = 0;
				numberUsedNodes = 1;

				bvhNodes[rootNodeIndex].leftFirst = 0; //No left child
				bvhNodes[rootNodeIndex].nrPrimitives = nrTriangles; 

				UpdateAABB(rootNodeIndex);
				Subdivide(rootNodeIndex);
			}

			float EvaluateSAH(const BVHNode& node, const int axis, const float position)
			{
				Aabb leftBox{}, rightBox{};
				int leftCount{}, rightCount{};

				for (size_t i{}; i < node.nrPrimitives; i++)
				{
					const size_t index{ node.leftFirst + i };
					const Vector3 centroid{ (transformedPositions[indices[index * 3]] + transformedPositions[indices[index * 3 + 1]] + transformedPositions[indices[index * 3 + 2]]) / 3.f };

					if (centroid[axis] < position)
					{
						leftCount++;

						leftBox.grow(transformedPositions[indices[index * 3]]);
						leftBox.grow(transformedPositions[indices[index * 3 + 1]]);
						leftBox.grow(transformedPositions[indices[index * 3 + 2]]);
					}
					else
					{
						rightCount++;

						rightBox.grow(transformedPositions[indices[index * 3]]);
						rightBox.grow(transformedPositions[indices[index * 3 + 1]]);
						rightBox.grow(transformedPositions[indices[index * 3 + 2]]);
					}
				}
				float cost = leftCount * leftBox.area() + rightCount * rightBox.area();
				return cost > 0 ? cost : INFINITY;
			}

			void Subdivide(const uint32_t nodeIdx)
			{
				BVHNode& node{ bvhNodes[nodeIdx] };

				int bestAxis{ -1 };
				float bestPos{ 0 };
				float bestCost{ INFINITY };

				for (int axis{}; axis < 3; axis++)
				{
					for (size_t i = 0; i < node.nrPrimitives; i++)
					{
						const size_t index{ node.leftFirst + i };
						const Vector3 centroid{ (transformedPositions[indices[index * 3]] + transformedPositions[indices[index * 3 + 1]] + transformedPositions[indices[index * 3 + 2]]) / 3.f };

						const float candidatePos = centroid[axis];
						const float cost{ EvaluateSAH(node, axis, candidatePos) };

						if (cost < bestCost)
						{
							bestPos = candidatePos;
							bestAxis = axis;
							bestCost = cost;
						}
					}
				}

				const Vector3 extent = node.maxAABB - node.minAABB; // extent of parent
				const float parentArea = extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
				const float parentCost = node.nrPrimitives * parentArea;

				if (bestCost >= parentCost) return;

				//Quicksort
				int left = node.leftFirst;
				const int right = left + node.nrPrimitives - 1;

				SortPrimitives(left, right, bestAxis, bestPos);

				int leftCount = left - node.leftFirst;

				if (leftCount == 0 || leftCount == node.nrPrimitives) return; 

				//Child nodes
				int leftChildIdx = numberUsedNodes;
				numberUsedNodes += 2;

				bvhNodes[leftChildIdx].leftFirst = node.leftFirst;
				bvhNodes[leftChildIdx].nrPrimitives = leftCount;
				bvhNodes[leftChildIdx + 1].leftFirst = left;
				bvhNodes[leftChildIdx + 1].nrPrimitives = node.nrPrimitives - leftCount;

				node.nrPrimitives = 0; //Is not leaf 
				node.leftFirst = leftChildIdx;

				UpdateAABB(leftChildIdx);
				UpdateAABB(leftChildIdx + 1);

				Subdivide(leftChildIdx);
				Subdivide(leftChildIdx + 1);
			}

			void SortPrimitives(int& left, int right, int axis, float splitPosition)
			{
				while (left <= right)
				{
					const Vector3 centroid{ (transformedPositions[indices[left * 3]] + transformedPositions[indices[left * 3 + 1]] + transformedPositions[indices[left * 3 + 2]]) / 3.f };
					if (centroid[axis] < splitPosition)
					{
						left++;
					}
					else
					{
						std::swap(normals[left], normals[right]);
						std::swap(transformedNormals[left], transformedNormals[right]);

						for (int i = 0; i < 3; i++)
						{
							std::swap(indices[left * 3 + i], indices[right * 3 + i]);
						}
						--right;
					}
				}
			}

		};

#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin{};
		Vector3 direction{};
		ColorRGB color{};
		float intensity{};

		LightType type{};
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Vector3 origin{};
		Vector3 direction{};
		Vector3 inverseDirection{};

		float min{ 0.0001f };
		float max{ FLT_MAX };
	};

	struct HitRecord
	{
		Vector3 origin{};
		Vector3 normal{};
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}