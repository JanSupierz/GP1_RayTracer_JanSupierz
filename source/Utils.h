#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
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
			const float factor{1 / (2.f * a) };

			float calculatedT{ (-b - sqrtDiscriminant) * factor };
			float secondT{ (-b + sqrtDiscriminant) * factor };
			
			if (secondT < calculatedT ) std::swap(calculatedT, secondT);

			if (calculatedT < 0)
			{
				calculatedT = secondT;
				if (calculatedT < 0)
				{
					return false;
				}
			}

			if ((calculatedT < ray.min || calculatedT > ray.max))
			{
				return false;
			}

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

			if (calculatedT < ray.min || calculatedT > ray.max)
			{
				return false;
			}
			
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
					if (dot > 0) return false;
				}
				else
				{
					if (dot < 0) return false;
				}
				break;
			case TriangleCullMode::FrontFaceCulling:
				if (ignoreHitRecord)
				{
					if (dot < 0) return false;
				}
				else
				{
					if (dot > 0) return false;
				}
				break;
			case TriangleCullMode::NoCulling:
				if (dot == 0) return false;
				break;

			}

			const float inverseDot{ 1.f / dot };
			const Vector3 originVector{ ray.origin - triangle.v0 };

			const float firstCalculation{ inverseDot * Vector3::Dot(originVector, rayDirectionAndEdge2Cross) };

			if (firstCalculation < 0.f || firstCalculation > 1.f)
			{
				return false;
			}

			const Vector3 rayOriginAndEdge1Cross{ Vector3::Cross(originVector, edge1) };
			const float secondCalculation{ inverseDot * Vector3::Dot(ray.direction, rayOriginAndEdge1Cross) };

			if (secondCalculation < 0.f || firstCalculation + secondCalculation > 1.f)
			{
				return false;
			}

			const float calculatedT{ inverseDot * Vector3::Dot(edge2, rayOriginAndEdge1Cross) };

			if (calculatedT < ray.min || calculatedT > ray.max)
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
		inline bool SlabTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{

			float tx1{ (mesh.transformedMinAABB.x - ray.origin.x) / ray.direction.x };
			float tx2{ (mesh.transformedMaxAABB.x - ray.origin.x) / ray.direction.x };

			float tmin{ std::min(tx1,tx2) };
			float tmax{ std::max(tx1,tx2) };

			float ty1{ (mesh.transformedMinAABB.y - ray.origin.y) / ray.direction.y };
			float ty2{ (mesh.transformedMaxAABB.y - ray.origin.y) / ray.direction.y };

			tmin = std::max(tmin,std::min(ty1,ty2));
			tmax = std::min(tmax,std::max(ty1,ty2));

			float tz1{ (mesh.transformedMinAABB.z - ray.origin.z) / ray.direction.z };
			float tz2{ (mesh.transformedMaxAABB.z - ray.origin.z) / ray.direction.z };
																				  
			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}
		
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			if (!SlabTest_TriangleMesh(mesh, ray))
			{
				return false;
			}

			Triangle triangle{};

			triangle.cullMode = mesh.cullMode;
			triangle.materialIndex = mesh.materialIndex;

			for (int index{}; index < mesh.normals.size(); ++index)
			{
				const int indicesIndex{ index * 3 };

				triangle.v0 = mesh.transformedPositions[mesh.indices[indicesIndex]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[indicesIndex + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[indicesIndex + 2]];

				triangle.normal = mesh.transformedNormals[index];

				if (HitTest_Triangle(triangle, ray, hitRecord, ignoreHitRecord) && ignoreHitRecord) return true; //Als het een schaduw is, dan mag je direct stoppen.
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