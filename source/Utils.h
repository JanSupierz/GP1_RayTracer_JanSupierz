#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"
#include <stack>

namespace dae
{
	namespace GeometryUtils
	{

		inline float SlabTest_BoundingBox(const Vector3& minAABB, const Vector3& maxAABB, const Ray& ray)
		{
			float tx1 = (minAABB.x - ray.origin.x) * ray.inverseDirection.x;
			float tx2 = (maxAABB.x - ray.origin.x) * ray.inverseDirection.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (minAABB.y - ray.origin.y)  * ray.inverseDirection.y;
			float ty2 = (maxAABB.y - ray.origin.y)  * ray.inverseDirection.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			float tz1 = (minAABB.z - ray.origin.z)  * ray.inverseDirection.z;
			float tz2 = (maxAABB.z - ray.origin.z)  * ray.inverseDirection.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;

			//return (tmax > 0 && tmax >= tmin) ? tmin : INFINITY;
		}

#pragma region Sphere HitTest

		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3 originVector{ ray.origin - sphere.origin };

			const float a{ Vector3::Dot(ray.direction, ray.direction) };
			const float b{ 2.f * Vector3::Dot(ray.direction, originVector) };
			const float c{ (Vector3::Dot(originVector,originVector)) - (sphere.radius * sphere.radius) };

			const float discriminant{ b * b - 4 * a * c };

			if (discriminant < 0.f)
			{
				return false;
			}

			const float sqrtDiscriminant{ sqrt(discriminant) };
			const float factor{ 1 / (2.f * a) };

			float calculatedT{ (-b - sqrtDiscriminant) * factor };
			float secondT{ (-b + sqrtDiscriminant) * factor };

			if (secondT < calculatedT) std::swap(calculatedT, secondT);

			if (calculatedT < 0)
			{
				calculatedT = secondT;
				if (calculatedT < 0)
				{
					return false;
				}
			}

			if (calculatedT >= ray.min && calculatedT <= ray.max)
			{
				if (ignoreHitRecord) return true;

				if (calculatedT < hitRecord.t)
				{
					hitRecord.t = calculatedT;

					hitRecord.materialIndex = sphere.materialIndex;
					hitRecord.didHit = true;
					hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
					hitRecord.normal = hitRecord.origin - sphere.origin;
					hitRecord.normal.Normalize();
				}

				return true;
			}

			return false;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}

#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const float calculatedT{ Vector3::Dot(plane.origin - ray.origin, plane.normal) / Vector3::Dot(ray.direction,plane.normal) };

			if (calculatedT >= ray.min && calculatedT <= ray.max)
			{
				if (ignoreHitRecord) return true;

				if (calculatedT < hitRecord.t)
				{
					hitRecord.t = calculatedT;

					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.didHit = true;
					hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
					hitRecord.normal = plane.normal;
				}

				return true;
			}

			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion

#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3 edge1{ triangle.v1 - triangle.v0 };
			const Vector3 edge2{ triangle.v2 - triangle.v0 };

			const Vector3 rayDirectionAndEdge2Cross = Vector3::Cross(ray.direction, edge2);

			const float dot{ Vector3::Dot(edge1, rayDirectionAndEdge2Cross) };

			switch (triangle.cullMode)
			{
			case TriangleCullMode::BackFaceCulling:
				if (ignoreHitRecord) 
				{
					if (dot > 0)
					{
						return false;
					}
				}
				else if(dot < 0)
				{
					return false;
				}
				break;
			case TriangleCullMode::FrontFaceCulling:
				if (ignoreHitRecord)
				{
					if (dot < 0)
					{
						return false;
					}
				}
				else if (dot > 0)
				{
					return false;
				}
				break;
			case TriangleCullMode::NoCulling:
				if (dot == 0) return false;
				break;

			}

			const float inverseDot{ 1.f / dot };
			const Vector3 originVector{ ray.origin - triangle.v0 };

			const float firstCalculation{ inverseDot * Vector3::Dot(originVector, rayDirectionAndEdge2Cross) };

			if (firstCalculation < 0.f || 1.f < firstCalculation)
			{
				return false;
			}

			const Vector3 rayOriginAndEdge1Cross{ Vector3::Cross(originVector, edge1) };
			const float secondCalculation{ inverseDot * Vector3::Dot(ray.direction, rayOriginAndEdge1Cross) };

			if (secondCalculation < 0.f ||  1.f < firstCalculation + secondCalculation)
			{
				return false;
			}

			const float calculatedT{ inverseDot * Vector3::Dot(edge2, rayOriginAndEdge1Cross) };

			if (calculatedT < ray.min || ray.max < calculatedT)
			{
				return false;
			}

			if (ignoreHitRecord) return true;

			if (calculatedT < hitRecord.t)
			{
				hitRecord.t = calculatedT;

				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.didHit = true;
				hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
				hitRecord.normal = triangle.normal;
			}

			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest

		inline void IntersectBVH(const Ray& ray,const TriangleMesh& mesh, const uint32_t nodeIdx, std::vector<int>& indices)
		{
			const BVHNode& node = mesh.bvhNodes[nodeIdx];

			if (!SlabTest_BoundingBox(node.minAABB, node.maxAABB, ray))
				return;

			if (node.nrPrimitives != 0) //Leaf
			{
				indices.push_back(nodeIdx);
				return;
			}
			else
			{
				IntersectBVH(ray, mesh, node.leftFirst, indices);
				IntersectBVH(ray, mesh, node.leftFirst + 1, indices);
			}
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//uint32_t indexToCheck{ mesh.rootNodeIdx };
			//std::stack<int> nodesToCheck{};

			//Triangle triangle{};
			//triangle.cullMode = mesh.cullMode;
			//triangle.materialIndex = mesh.materialIndex;

			//while (1)
			//{
			//	if (mesh.bvhNodes[indexToCheck].nrPrimitives != 0) //is leaf
			//	{
			//		int nrVertices{ 3 };
			//		uint32_t start = mesh.bvhNodes[indexToCheck].leftFirst;
			//		uint32_t end = start + mesh.bvhNodes[indexToCheck].nrPrimitives;

			//		for (uint32_t currentTriangle = start; currentTriangle < end; ++currentTriangle)
			//		{
			//			triangle.v0 = mesh.transformedPositions[mesh.indices[currentTriangle * nrVertices]];
			//			triangle.v1 = mesh.transformedPositions[mesh.indices[currentTriangle * nrVertices + 1]];
			//			triangle.v2 = mesh.transformedPositions[mesh.indices[currentTriangle * nrVertices + 2]];
			//			triangle.normal = mesh.transformedNormals[currentTriangle];

			//			if (HitTest_Triangle(triangle, ray, hitRecord) && ignoreHitRecord) return true;
			//		}

			//		if (nodesToCheck.size() == 0)
			//		{
			//			//is de laatste node die gecheckt moest worden
			//			break;
			//		}
			//		else
			//		{
			//			//we checken de volgende node op de stack
			//			indexToCheck = nodesToCheck.top();
			//			nodesToCheck.pop();
			//		}

			//	}
			//	else
			//	{
			//		int firstChildIndex = mesh.bvhNodes[indexToCheck].leftFirst;
			//		int secondChildIndex = firstChildIndex + 1;

			//		float dist1 = SlabTest_BoundingBox(mesh.bvhNodes[firstChildIndex].minAABB, mesh.bvhNodes[firstChildIndex].maxAABB, ray);
			//		float dist2 = SlabTest_BoundingBox(mesh.bvhNodes[secondChildIndex].minAABB, mesh.bvhNodes[secondChildIndex].maxAABB, ray);

			//		if (dist1 > dist2)
			//		{
			//			std::swap(dist1, dist2); //als de 1ste node verder is dan de 2de, swap de volgorde
			//			std::swap(firstChildIndex, secondChildIndex);
			//		}

			//		if (dist1 == INFINITY) //geen hit met dichtere boundingbox
			//		{
			//			if (nodesToCheck.size() == 0)
			//			{
			//				//is de laatste node die gecheckt moest worden
			//				break;
			//			}
			//			else
			//			{
			//				//we checken de rest van nodes op de stack
			//				indexToCheck = nodesToCheck.top();
			//				nodesToCheck.pop();
			//			}

			//		}
			//		else
			//		{
			//			//als we de boundingbox hitten
			//			indexToCheck = firstChildIndex; //dan gaan we nu kijken of we de triangle hitten
			//			if (dist2 != INFINITY)
			//			{
			//				//de tweede kan misschien ook gehit worden
			//				nodesToCheck.push(secondChildIndex);//we voegen het op de stack
			//			}
			//		}
			//	}
			//}
				
			std::vector<int> indices{};
			IntersectBVH(ray, mesh, mesh.rootNodeIdx, indices);

			if (indices.size() == 0)
				return hitRecord.didHit;

			Triangle triangle{};
			triangle.cullMode = mesh.cullMode;
			triangle.materialIndex = mesh.materialIndex;

			for (size_t i = 0; i < indices.size(); i++)
			{
				int nrVertices{ 3 };
				uint32_t start = mesh.bvhNodes[indices[i]].leftFirst;
				uint32_t end = start + mesh.bvhNodes[indices[i]].nrPrimitives;

				for (uint32_t currentTriangle = start; currentTriangle < end; ++currentTriangle)
				{
					triangle.v0 = mesh.transformedPositions[mesh.indices[currentTriangle * nrVertices]];
					triangle.v1 = mesh.transformedPositions[mesh.indices[currentTriangle * nrVertices + 1]];
					triangle.v2 = mesh.transformedPositions[mesh.indices[currentTriangle * nrVertices + 2]];
					triangle.normal = mesh.transformedNormals[currentTriangle];

					if (HitTest_Triangle(triangle, ray, hitRecord) && ignoreHitRecord) return true;
				}
			}

			return hitRecord.didHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			return { light.origin - origin };
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			return { light.color * light.intensity / (GetDirectionToLight(light, target).SqrMagnitude()) };
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}