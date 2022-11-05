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
			Vector3 bMin = Vector3{ INFINITY,INFINITY,INFINITY }, bMax = -Vector3{ INFINITY,INFINITY,INFINITY };

			void grow(Vector3 p) 
			{ 
				bMin.x = std::min(bMin.x, p.x);
				bMin.y = std::min(bMin.y, p.y);
				bMin.z = std::min(bMin.z, p.z);

				bMax.x = std::min(bMax.x, p.x);
				bMax.y = std::min(bMax.y, p.y);
				bMax.z = std::min(bMax.z, p.z);
			}

			float area()
			{
				Vector3 extent = bMax - bMin; // box extent
				return extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
			}
		};

		struct TriangleMesh
		{
			TriangleMesh() = default;
			TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode) :
				positions(_positions), indices(_indices), cullMode(_cullMode)
			{
				trCount = static_cast<int>(_indices.size()) / 3;
				CalculateNormals();
				CalculateCentroids();
				InitBVH();

				//Update Transforms
				//UpdateTransforms();
			}

			TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
				positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
			{
				trCount = static_cast<int>(_indices.size()) / 3;
				CalculateCentroids();
				InitBVH();
				//UpdateTransforms();
			}

			std::vector<Vector3> positions{};
			std::vector<Vector3> normals{};
			std::vector<Vector3> centroids{};
			std::vector<int> indices{};
			unsigned char materialIndex{};
			uint32_t trCount{};

			TriangleCullMode cullMode{ TriangleCullMode::BackFaceCulling };

			Matrix rotationTransform{};
			Matrix translationTransform{};
			Matrix scaleTransform{};

			Vector3 minAABB;
			Vector3 maxAABB;

			Vector3 transformedMinAABB;
			Vector3 transformedMaxAABB;

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

				//Not ideal, but making sure all vertices are updated
				if (!ignoreTransformUpdate)
					UpdateTransforms();
			}

			void CalculateNormals()
			{
				if (trCount % 3 != 0)
				{
					return;
				}

				Vector3 v0{};
				Vector3 v1{};
				Vector3 v2{};

				for (size_t currentTriangle = 0; currentTriangle < trCount; ++currentTriangle)
				{
					v0 = positions[indices[currentTriangle * 3]];
					v1 = positions[indices[currentTriangle * 3 + 1]];
					v2 = positions[indices[currentTriangle * 3 + 2]];

					normals.push_back(Vector3::Cross(v1 - v0, v2 - v0).Normalized());
				}

			}

			void CalculateCentroids()
			{
				trCount = static_cast<int>(indices.size()) / 3;
				centroids.reserve(normals.size());
				Vector3 v0{};
				Vector3 v1{};
				Vector3 v2{};

				for (size_t currentTriangle = 0; currentTriangle < trCount; ++currentTriangle)
				{
					v0 = positions[indices[currentTriangle * 3]];
					v1 = positions[indices[currentTriangle * 3 + 1]];
					v2 = positions[indices[currentTriangle * 3 + 2]];

					centroids.push_back((v0 + v1 + v2) / 3.f);
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

				auto transformMatrix = translationTransform * rotationTransform * scaleTransform;

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


				//UpdateTransformedAABB(transformMatrix);
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

			void UpdateTransformedAABB(const Matrix& finalTransform)
			{
				//Transform the 8 vertices of the aabb
				//then calculate new min and max
				Vector3 tMinAABB = finalTransform.TransformPoint(minAABB);
				Vector3 tMaxAABB = tMinAABB;

				// (xmax,ymin,zmin)
				Vector3 tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z);
				tMinAABB = Vector3::Min(tAABB, tMinAABB);
				tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

				// (xmax,ymin,zmax)
				tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
				tMinAABB = Vector3::Min(tAABB, tMinAABB);
				tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

				// (xmin,ymin,zmax)
				tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
				tMinAABB = Vector3::Min(tAABB, tMinAABB);
				tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

				// (xmin,ymax,zmin)
				tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
				tMinAABB = Vector3::Min(tAABB, tMinAABB);
				tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

				// (xmax,ymax,zmin)
				tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
				tMinAABB = Vector3::Min(tAABB, tMinAABB);
				tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

				// (xmax,ymax,zmax)
				tAABB = finalTransform.TransformPoint(maxAABB);
				tMinAABB = Vector3::Min(tAABB, tMinAABB);
				tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

				// (xmin,ymax,zmax)
				tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
				tMinAABB = Vector3::Min(tAABB, tMinAABB);
				tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

				transformedMinAABB = tMinAABB;
				transformedMaxAABB = tMaxAABB;

			}

			void InitBVH()
			{
				bvhNodes.reserve(trCount * 2 - 1);

				for (size_t i = 0; i < bvhNodes.capacity(); i++)
				{
					bvhNodes.push_back(BVHNode{});
				}

				rootNodeIdx = 0;
				nodesUsed = 1;

				bvhNodes[rootNodeIdx].leftFirst = 0; //Means no left child
				bvhNodes[rootNodeIdx].nrPrimitives = trCount; //IsLeaf

				UpdateAABB(rootNodeIdx);
				Subdivide(rootNodeIdx);
			}

			float EvaluateSAH(BVHNode& node, int axis, float pos)
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
						leftBox.grow(transformedPositions[index]);
						leftBox.grow(transformedPositions[index+1]);
						leftBox.grow(transformedPositions[index+2]);
					}
					else
					{
						rightCount++;
						rightBox.grow(transformedPositions[index]);
						rightBox.grow(transformedPositions[index+1]);
						rightBox.grow(transformedPositions[index+2]);
					}
				}
				float cost = leftCount * leftBox.area() + rightCount * rightBox.area();
				return cost > 0 ? cost : INFINITY;
			}

			void Subdivide(uint32_t nodeIdx)
			{
				BVHNode& node = bvhNodes[nodeIdx];
				//Terminate recursion
				if (node.nrPrimitives <= 2)
					return;

				// determine split axis using SAH
				//int bestAxis = -1;
				//float bestPos = 0, bestCost = INFINITY;

				//for (int axis = 0; axis < 3; axis++) for (size_t i = 0; i < node.nrPrimitives; i++)
				//{
				//	size_t index = node.leftFirst + i;
				//	float candidatePos = transformedCentroids[index][axis];
				//	float cost = EvaluateSAH(node, axis, candidatePos);
				//	if (cost < bestCost)
				//	{
				//		bestPos = candidatePos, bestAxis = axis, bestCost = cost;
				//	}
				//}
				//int axis = bestAxis;
				//float splitPos = bestPos;

				//const Vector3 extent = node.maxAABB - node.minAABB; // extent of parent
				//float parentArea = extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
				//float parentCost = node.nrPrimitives * parentArea;

				//if (bestCost >= parentCost) return;

				Vector3 axisSize = node.maxAABB - node.minAABB;

				//We subdevide along the longest axis
				int axis = 0; //x-axis
				if (axisSize.y > axisSize.x)
				{
					axis = 1; // y-axis
				}
				if (axisSize.z > axisSize[axis])
				{
					axis = 2; //z-axis
				}
				bool testedAllAxis{ false };
				float splitPos{ node.minAABB[axis] + axisSize[axis] / 2.0f };

				//Sort the primitives (Quicksort)
				int left = node.leftFirst;
				int right = left + node.nrPrimitives - 1;

				int leftCount{};
				int sortCount{ 0 };

				while (sortCount < 3)
				{
					++sortCount;

					left = node.leftFirst;
					SortPrimitives(left, right, axis, splitPos);
					leftCount = left - node.leftFirst;

					if (axis == 2)
					{
						axis = 0;
					}
					else
					{
						++axis;
					}

					if (leftCount != 0 && leftCount != node.nrPrimitives)
					{
						break;
					}
				}

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