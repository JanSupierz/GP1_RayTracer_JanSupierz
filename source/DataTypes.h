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
				centroid = (_v0 + _v1 + _v2) / 3.0f;
			}

			Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
				v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
			{
				const Vector3 edgeV0V1 = v1 - v0;
				const Vector3 edgeV0V2 = v2 - v0;
				normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
				centroid = (_v0 + _v1 + _v2) / 3.0f;
			}

			Vector3 v0{};
			Vector3 v1{};
			Vector3 v2{};

			Vector3 normal{};
			Vector3 centroid{};

			TriangleCullMode cullMode{};
			unsigned char materialIndex{};
		};

		//Implemented using https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/

		struct BVHNode
		{
			Vector3 minAABB, maxAABB;
			uint32_t leftFirst, nrPrimitives;
		};

		struct aabb
		{
			Vector3 min = Vector3{ INFINITY,INFINITY,INFINITY }, max = -Vector3{ INFINITY,INFINITY,INFINITY };

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
				nrTriangles = static_cast<int>(_indices.size()) / 3;
			}

			std::vector<Vector3> positions{};
			std::vector<Vector3> normals{};
			std::vector<Vector3> centroids{};
			std::vector<int> indices{};

			unsigned char materialIndex{};

			uint32_t nrTriangles{};

			TriangleCullMode cullMode{ TriangleCullMode::BackFaceCulling };

			Matrix rotationTransform{};
			Matrix translationTransform{};
			Matrix scaleTransform{};

			std::vector<Vector3> transformedPositions{};
			std::vector<Vector3> transformedNormals{};
			std::vector<Vector3> transformedCentroids{};

			std::vector<BVHNode> bvhNodes{};
			uint32_t rootNodeIdx{};
			uint32_t nodesUsed{};

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
				centroids.push_back(triangle.centroid);

				//Not ideal, but making sure all vertices are updated
				if (!ignoreTransformUpdate)
					UpdateTransforms();
			}

			void CalculateNormals()
			{
				if (nrTriangles % 3 != 0)
				{
					return;
				}

				Vector3 v0{};
				Vector3 v1{};
				Vector3 v2{};

				for (size_t currentTriangle = 0; currentTriangle < nrTriangles; ++currentTriangle)
				{
					v0 = positions[indices[currentTriangle * 3]];
					v1 = positions[indices[currentTriangle * 3 + 1]];
					v2 = positions[indices[currentTriangle * 3 + 2]];

					normals.push_back(Vector3::Cross(v1 - v0, v2 - v0).Normalized());
				}

			}

			void CalculateCentroids()
			{
				const float division{ 1.f / 3.f };

				nrTriangles = static_cast<int>(indices.size() * division);
				centroids.reserve(normals.size());
				Vector3 v0{};
				Vector3 v1{};
				Vector3 v2{};

				for (size_t currentTriangle = 0; currentTriangle < nrTriangles; ++currentTriangle)
				{
					v0 = positions[indices[currentTriangle * 3]];
					v1 = positions[indices[currentTriangle * 3 + 1]];
					v2 = positions[indices[currentTriangle * 3 + 2]];

					centroids.push_back((v0 + v1 + v2) * division);
				}
			}

			void UpdateTransforms()
			{
				transformedNormals.clear();
				transformedPositions.clear();
				transformedCentroids.clear();
				transformedNormals.reserve(normals.size());
				transformedCentroids.reserve(centroids.size());
				transformedPositions.reserve(positions.size());

				auto transformMatrix = rotationTransform * translationTransform * scaleTransform;

				for (size_t i = 0; i < positions.size(); i++)
				{
					transformedPositions.emplace_back(transformMatrix.TransformPoint(positions[i]));
				}

				for (size_t i = 0; i < centroids.size(); i++)
				{
					transformedCentroids.emplace_back(transformMatrix.TransformPoint(centroids[i]));
				}

				for (size_t i = 0; i < normals.size(); i++)
				{
					transformedNormals.emplace_back((transformMatrix.TransformVector(normals[i])).Normalized());
				}

				RefitBVH();
			}

			void RefitBVH()
			{
				for (int i = nodesUsed - 1; i >= 0; i--)
				{
					BVHNode& node = bvhNodes[i];

					if (node.nrPrimitives != 0)
					{
						UpdateAABB(i);
						continue;
					}

					BVHNode& leftChild = bvhNodes[node.leftFirst];
					BVHNode& rightChild = bvhNodes[node.leftFirst + 1];
					node.minAABB = Vector3::Min(leftChild.minAABB, rightChild.minAABB);
					node.maxAABB = Vector3::Max(leftChild.maxAABB, rightChild.maxAABB);
				}
			}

			void UpdateAABB(uint32_t nodeIdx)
			{
				BVHNode& node = bvhNodes[nodeIdx];

				uint32_t start{ node.leftFirst * 3 };
				uint32_t end{ start + node.nrPrimitives * 3 };

				node.minAABB = Vector3{ INFINITY,INFINITY,INFINITY };
				node.maxAABB = Vector3{ -INFINITY,-INFINITY,-INFINITY };

				for (uint32_t i = start; i < end; ++i)
				{
					node.minAABB = Vector3::Min(transformedPositions[indices[i]], node.minAABB);
					node.maxAABB = Vector3::Max(transformedPositions[indices[i]], node.maxAABB);
				}
			}

			void InitBVH()
			{
				bvhNodes.reserve(nrTriangles * 2 - 1);

				for (size_t i = 0; i < bvhNodes.capacity(); i++)
				{
					bvhNodes.push_back(BVHNode{});
				}

				rootNodeIdx = 0;
				nodesUsed = 1;

				bvhNodes[rootNodeIdx].leftFirst = 0; //Means no left child
				bvhNodes[rootNodeIdx].nrPrimitives = nrTriangles; //IsLeaf

				UpdateAABB(rootNodeIdx);
				Subdivide(rootNodeIdx);
			}

			float EvaluateSAH(const BVHNode& node, const int axis, const float pos)
			{
				// determine triangle counts and bounds for this split candidate
				aabb leftBox, rightBox;
				int leftCount = 0, rightCount = 0;
				for (size_t i = 0; i < node.nrPrimitives; i++)
				{
					size_t index = node.leftFirst + i;
					if (transformedCentroids[index][axis] < pos)
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
				BVHNode& node = bvhNodes[nodeIdx];

				// determine split axis using SAH
				int bestAxis = -1;
				float bestPos = 0;
				float bestCost = INFINITY;

				for (int axis = 0; axis < 3; axis++)
				{
					for (size_t i = 0; i < node.nrPrimitives; i++)
					{
						float candidatePos = transformedCentroids[node.leftFirst + i][axis];

						float cost = EvaluateSAH(node, axis, candidatePos);

						if (cost < bestCost)
						{
							bestPos = candidatePos;
							bestAxis = axis;
							bestCost = cost;
						}
					}
				}

				const Vector3 extent = node.maxAABB - node.minAABB; // extent of parent
				float parentArea = extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
				float parentCost = node.nrPrimitives * parentArea;

				if (bestCost >= parentCost) return;

				//Sort the primitives (Quicksort)
				int left = node.leftFirst;
				int right = left + node.nrPrimitives - 1;

				SortPrimitives(left, right, bestAxis, bestPos);

				int leftCount = left - node.leftFirst;

				if (leftCount == 0 || leftCount == node.nrPrimitives)
					return; // Either nothing on the left side, or everything on the left side

				//Create child nodes
				int leftChildIdx = nodesUsed;
				nodesUsed += 2;

				bvhNodes[leftChildIdx].leftFirst = node.leftFirst; // Left side starts where the parent's left side starts
				bvhNodes[leftChildIdx].nrPrimitives = leftCount;
				bvhNodes[leftChildIdx + 1].leftFirst = left; // While loop made sure this is where the right side starts
				bvhNodes[leftChildIdx + 1].nrPrimitives = node.nrPrimitives - leftCount; //remaining primitives belong to the right side

				node.nrPrimitives = 0; //Is not leaf 
				node.leftFirst = leftChildIdx;

				UpdateAABB(leftChildIdx);
				UpdateAABB(leftChildIdx + 1);

				Subdivide(leftChildIdx);
				Subdivide(leftChildIdx + 1);
			}

			void SortPrimitives(int& left, int right, int axis, float splitPos)
			{
				while (left <= right)
				{
					//If the centeroid is on the left side --> OK
					if (transformedCentroids[left][axis] < splitPos)
					{
						left++;
					}
					else
					{
						//Move to the end of the container
						std::swap(transformedCentroids[left], transformedCentroids[right]);
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