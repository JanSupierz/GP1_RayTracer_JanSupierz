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

			if (discriminant < 0.f )
			{
				return false;
			}
			else
			{
				const float sqrtDiscriminant{ sqrt(discriminant) };

				float calculatedT{};

				if (discriminant > 0.f)
				{
					float firstT{ (-b + sqrtDiscriminant) / (2.f * a) };
					float secondT{ (-b - sqrtDiscriminant) / (2.f * a) };
					

					if (firstT > 0.f && secondT > 0.f)
					{
						calculatedT = (firstT < secondT ? firstT : secondT);
					}
					else 
					{
						calculatedT = (firstT > secondT ? firstT : secondT);
					}
		
				}
				else
				{
					calculatedT = (-b + sqrtDiscriminant) / (2.f * a);
				}

				if ((calculatedT >= ray.min && calculatedT <= ray.max))
				{
					if (ignoreHitRecord) return true;

					if (hitRecord.t > calculatedT)
					{
						hitRecord.t = calculatedT;

						hitRecord.materialIndex = sphere.materialIndex;
						hitRecord.didHit = true;
						hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
						hitRecord.normal = hitRecord.origin - sphere.origin;
						hitRecord.normal.Normalize();
					}
				}
			}

			return hitRecord.didHit;
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

				if (hitRecord.t > calculatedT)
				{
					hitRecord.t = calculatedT;

					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.didHit = true;
					hitRecord.origin = ray.origin + ray.direction * hitRecord.t;
					hitRecord.normal = plane.normal;	
				}
			}

			return hitRecord.didHit;
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
			const float dot{ Vector3::Dot(triangle.normal, ray.direction) };

			if (dot == 0) return false;

			switch (triangle.cullMode)
			{
			case TriangleCullMode::BackFaceCulling:
				if (ignoreHitRecord)
				{
					if (dot < 0) return false;
				}
				else
				{
					if (dot > 0) return false;
				}
				break;
			case TriangleCullMode::FrontFaceCulling:
				if (ignoreHitRecord)
				{
					if (dot > 0) return false;
				}
				else
				{
					if (dot < 0) return false;
				}
				break;
			}

			const Vector3 triangleCenter{ (triangle.v0 + triangle.v1 + triangle.v2) / 3.f };

			float calculatedT{ Vector3::Dot(triangleCenter - ray.origin,triangle.normal) / Vector3::Dot(ray.direction,triangle.normal) };

			if (calculatedT >= ray.min && calculatedT <= ray.max)
			{
				if (hitRecord.t > calculatedT)
				{
					const Vector3 point{ ray.origin + calculatedT * ray.direction };

					Vector3 triangleEdge{ triangle.v1 - triangle.v0 };
					Vector3 pointToSide{ point - triangle.v0 };

					if (Vector3::Dot(triangle.normal, Vector3::Cross(triangleEdge, pointToSide)) < 0.f) return false;

					triangleEdge = triangle.v2 - triangle.v1;
					pointToSide = point - triangle.v1;

					if (Vector3::Dot(triangle.normal, Vector3::Cross(triangleEdge, pointToSide)) < 0.f) return false;

					triangleEdge = triangle.v0 - triangle.v2;
					pointToSide = point - triangle.v2;

					if (Vector3::Dot(triangle.normal, Vector3::Cross(triangleEdge, pointToSide)) < 0.f) return false;

					if (ignoreHitRecord) return true;

					hitRecord.t = calculatedT;

					hitRecord.materialIndex = triangle.materialIndex;
					hitRecord.didHit = true;
					hitRecord.origin = point;
					hitRecord.normal = triangle.normal;
				}
			}

			return hitRecord.didHit;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
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