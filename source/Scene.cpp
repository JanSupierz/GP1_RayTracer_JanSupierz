#include "Scene.h"
#include "Utils.h"
#include "Material.h"

namespace dae {

#pragma region Base Scene
	//Initialize Scene with Default Solid Color Material (RED)
	Scene::Scene():
		m_Materials({ new Material_SolidColor({1,0,0})})
	{
		m_SphereGeometries.reserve(32);
		m_PlaneGeometries.reserve(32);
		m_TriangleMeshGeometries.reserve(32);
		m_Lights.reserve(32);
	}

	Scene::~Scene()
	{
		for(auto& pMaterial : m_Materials)
		{
			delete pMaterial;
			pMaterial = nullptr;
		}

		m_Materials.clear();
	}

	void dae::Scene::GetClosestHit(const Ray& ray, HitRecord& closestHit) const
	{
		if (GeometryUtils::SlabTest_BoundingBox(m_SpheresBoundingBox.min, m_SpheresBoundingBox.max, ray))
		{
			for (const Sphere& sphere : m_SphereGeometries)
			{
				dae::GeometryUtils::HitTest_Sphere(sphere, ray, closestHit);
			}
		}
		
		if (GeometryUtils::SlabTest_BoundingBox(m_TrianglesBoundingBox.min, m_TrianglesBoundingBox.max, ray))
		{
			for (const TriangleMesh& triangleMesh : m_TriangleMeshGeometries)
			{
				dae::GeometryUtils::HitTest_TriangleMesh(triangleMesh, ray, closestHit);
			}
		}

		for (const Plane& plane : m_PlaneGeometries)
		{
			dae::GeometryUtils::HitTest_Plane(plane, ray, closestHit);
		}
	}

	bool Scene::DoesHit(const Ray& ray) const
	{
		if (GeometryUtils::SlabTest_BoundingBox(m_SpheresBoundingBox.min, m_SpheresBoundingBox.max, ray))
		{
			for (const Sphere& sphere : m_SphereGeometries)
			{
				if (dae::GeometryUtils::HitTest_Sphere(sphere, ray)) return true;
			}
		}

		if (GeometryUtils::SlabTest_BoundingBox(m_TrianglesBoundingBox.min, m_TrianglesBoundingBox.max, ray))
		{
			for (const TriangleMesh& triangleMesh : m_TriangleMeshGeometries)
			{
				if (dae::GeometryUtils::HitTest_TriangleMesh(triangleMesh, ray)) return true;
			}
		}
		
		for (const Plane& plane : m_PlaneGeometries)
		{
			if (dae::GeometryUtils::HitTest_Plane(plane, ray)) return true;
		}

		return false;
	}

#pragma region Scene Helpers

	Sphere* Scene::AddSphere(const Vector3& origin, float radius, unsigned char materialIndex)
	{
		Sphere s;
		s.origin = origin;
		s.radius = radius;
		s.materialIndex = materialIndex;

		m_SphereGeometries.emplace_back(s);
		return &m_SphereGeometries.back();
	}

	Plane* Scene::AddPlane(const Vector3& origin, const Vector3& normal, unsigned char materialIndex)
	{
		Plane p;
		p.origin = origin;
		p.normal = normal;
		p.materialIndex = materialIndex;

		m_PlaneGeometries.emplace_back(p);
		return &m_PlaneGeometries.back();
	}

	TriangleMesh* Scene::AddTriangleMesh(TriangleCullMode cullMode, unsigned char materialIndex)
	{
		TriangleMesh m{};
		m.cullMode = cullMode;
		m.materialIndex = materialIndex;

		m_TriangleMeshGeometries.emplace_back(m);
		return &m_TriangleMeshGeometries.back();
	}

	Light* Scene::AddPointLight(const Vector3& origin, float intensity, const ColorRGB& color)
	{
		Light l;
		l.origin = origin;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Point;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	Light* Scene::AddDirectionalLight(const Vector3& direction, float intensity, const ColorRGB& color)
	{
		Light l;
		l.direction = direction;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Directional;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	unsigned char Scene::AddMaterial(Material* pMaterial)
	{
		m_Materials.push_back(pMaterial);
		return static_cast<unsigned char>(m_Materials.size() - 1);
	}
#pragma endregion
#pragma endregion

#pragma region SCENE W1
	void Scene_W1::Initialize()
	{
		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });
		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });

		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Spheres
		AddSphere({ -25.f, 0.f, 100.f }, 50.f, matId_Solid_Red);
		AddSphere({ 25.f, 0.f, 100.f }, 50.f, matId_Solid_Blue);

		//Plane
		AddPlane({ -75.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 75.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 0.f, -75.f, 0.f }, { 0.f, 1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 75.f, 0.f }, { 0.f, -1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 0.f, 125.f }, { 0.f, 0.f,-1.f }, matId_Solid_Magenta);
	}
#pragma endregion

#pragma region SCENE W2
	void Scene_W2::Initialize()
	{
		m_Camera.origin = { 0.f,3.f,-9.f };
		m_Camera.fovAngle = 45.f;

		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Plane
		AddPlane({ -5.f,0.f,0.f }, { 1.f,0.f,0.f }, matId_Solid_Green);
		AddPlane({ 5.f,0.f,0.f }, { -1.f,0.f,0.f }, matId_Solid_Green);
		AddPlane({ 0,0.f,0.f }, { 0.f,1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0,10.f,0.f }, { 0.f,-1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0,0.f,10.f }, { 0.f,0.f,-1.f }, matId_Solid_Magenta);

		//Spheres
		AddSphere({ -1.75f,1.f,0.f }, .75f, matId_Solid_Red);
		AddSphere({ 0.f,1.f,0.f }, .75f, matId_Solid_Blue);
		AddSphere({ 1.75f,1.f,0.f }, .75f, matId_Solid_Red);
		AddSphere({ -1.75f,3.f,0.f }, .75f, matId_Solid_Blue);
		AddSphere({ 0.f,3.f,0.f }, .75f, matId_Solid_Red);
		AddSphere({ 1.75f,3.f,0.f }, .75f, matId_Solid_Blue);

		//Light
		AddPointLight({ 0.f, 5.f, -5.f }, 70.f, colors::White);
	}
#pragma endregion

#pragma region SCENE W3
	void Scene_W3_TestScene::Initialize()
	{
		m_Camera.origin = { 0.f, 1.f, -5.f };
		m_Camera.fovAngle = 45.f;

		//Lambert Materials
		const auto matLambert_Red = AddMaterial(new Material_Lambert{ colors::Red, 1.f });
		const auto matLambert_Blue = AddMaterial(new Material_Lambert{ colors::Blue, 1.f });
		const auto matLambert_Yellow = AddMaterial(new Material_Lambert{ colors::Yellow, 1.f });

		//Phong Material
		const auto matLambertPhong_Blue = AddMaterial(new Material_LambertPhong(colors::Blue, 1.f, 1.f, 60.f));

		//CookTorrence
		const auto matCookTorrence_Red = AddMaterial(new Material_CookTorrence(colors::Red, 0.f, 0.5f));

		//Spheres
		AddSphere({ -.75f, 1.f, .0f }, 1.f, matCookTorrence_Red);
		AddSphere({ .75f, 1.f, .0f }, 1.f, matLambertPhong_Blue);

		//Plane
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, matLambert_Yellow);

		//Light
		AddPointLight({ 0.f, 5.f, 5.f }, 25.f, colors::White);
		AddPointLight({ 0.f, 2.5f, -5.f }, 25.f, colors::White);
	}

	void Scene_W3::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;

		const auto matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({ 0.972f,0.960f,0.915f }, 1.f, 1.f));
		const auto matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ 0.972f,0.960f,0.915f }, 1.f, 0.6f));
		const auto matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ 0.972f,0.960f,0.915f }, 1.f, 0.1f));
		const auto matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ 0.75f,0.75f,0.75f }, 0.f, 1.f));
		const auto matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ 0.75f,0.75f,0.75f }, 0.f, 0.6f));
		const auto matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ 0.75f,0.75f,0.75f }, 0.f, 0.1f));

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ 0.49f,0.57f,0.57f }, 1.f));

		//Planes
		AddPlane(Vector3{ 0.f,0.f,10.f }, Vector3{ 0.f,0.f,-1.f }, matLambert_GrayBlue); //Back
		AddPlane(Vector3{ 0.f,0.f,0.f }, Vector3{ 0.f,1.f,0.f }, matLambert_GrayBlue); //Bottom
		AddPlane(Vector3{ 0.f,10.f,0.f }, Vector3{ 0.f,-1.f,0.f }, matLambert_GrayBlue); //Top
		AddPlane(Vector3{ 5.f,0.f,0.f }, Vector3{ -1.f,0.f,0.f }, matLambert_GrayBlue); //Right
		AddPlane(Vector3{ -5.f,0.f,0.f }, Vector3{ 1.f,0.f,0.f }, matLambert_GrayBlue); //Left

		//Temporary Lambert-Phong Spheres and Materials
		const auto matLambertPhong1 = AddMaterial(new Material_LambertPhong(colors::Blue, 0.5f, 0.5f, 3.f));
		const auto matLambertPhong2 = AddMaterial(new Material_LambertPhong(colors::Blue, 0.5f, 0.5f, 15.f));
		const auto matLambertPhong3 = AddMaterial(new Material_LambertPhong(colors::Blue, 0.5f, 0.5f, 50.f));
		
		//AddSphere(Vector3{ -1.75f,1.f,0.f }, 0.75f, matLambertPhong1);
		//AddSphere(Vector3{ 0.f,1.f,0.f }, 0.75f, matLambertPhong2);
		//AddSphere(Vector3{ 1.75f,1.f,0.f }, 0.75f, matLambertPhong3);

		//Spheres
		AddSphere(Vector3{ -1.75f,1.f,0.f }, 0.75f, matCT_GrayRoughMetal);
		AddSphere(Vector3{ 0.f,1.f,0.f }, 0.75f, matCT_GrayMediumMetal);
		AddSphere(Vector3{ 1.75f,1.f,0.f }, 0.75f, matCT_GraySmoothMetal);
		AddSphere(Vector3{ -1.75f,3.f,0.f }, 0.75f, matCT_GrayRoughPlastic);
		AddSphere(Vector3{ 0.f,3.f,0.f }, 0.75f, matCT_GrayMediumPlastic);
		AddSphere(Vector3{ 1.75f,3.f,0.f }, 0.75f, matCT_GraySmoothPlastic);

		//Light
		AddPointLight(Vector3{ 0.f,5.f,5.f }, 50.f, ColorRGB{ 1.f,0.61f,0.45f }); //Back light
		AddPointLight(Vector3{ -2.5f,5.f,-5.f }, 70.f, ColorRGB{ 1.f,0.8f,0.45f }); //Front light left
		AddPointLight(Vector3{ 2.5f,2.5f,-5.f }, 50.f, ColorRGB{ 0.34f,0.47f,0.68f });
	}
#pragma endregion

#pragma region SCENE W4
	void Scene_W4_TestScene::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;

		//Materials
		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ 0.49f,0.57f,0.57f }, 1.f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//Planes
		AddPlane(Vector3{ 0.f,0.f,10.f }, Vector3{ 0.f,0.f,-1.f }, matLambert_GrayBlue); //BACK
		AddPlane(Vector3{ 0.f,0.f,0.f }, Vector3{ 0.f,1.f,0.f }, matLambert_GrayBlue); //BOTTOM
		AddPlane(Vector3{ 0.f,10.f,0.f }, Vector3{ 0.f,-1.f,0.f }, matLambert_GrayBlue); //TOP
		AddPlane(Vector3{ 5.f,0.f,0.f }, Vector3{ -1.f,0.f,0.f }, matLambert_GrayBlue); //RIGHT
		AddPlane(Vector3{ -5.f,0.f,0.f }, Vector3{ 1.f,0.f,0.f }, matLambert_GrayBlue); //LEFT

		pMesh = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/simple_cube.obj", pMesh->positions, pMesh->normals, pMesh->indices);

		pMesh->Scale({ 0.7f,0.7f,0.7f });
		pMesh->Translate({ 0.f,1.f,0.f });
		pMesh->UpdateTransforms();

		pMesh->InitBVH();

		//Light
		AddPointLight(Vector3{ 0.f,5.f,5.f }, 50.f, ColorRGB{ 1.f,0.61f,0.45f }); //Back light
		AddPointLight(Vector3{ -2.5f,5.f,-5.f }, 70.f, ColorRGB{ 1.f,0.8f,0.45f }); //Front light left
		AddPointLight(Vector3{ 2.5f,2.5f,-5.f }, 50.f, ColorRGB{ 0.34f,0.47f,0.68f });
	}

	void Scene_W4_TestScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		pMesh->RotateY(PI_DIV_2 * pTimer->GetTotal());
		pMesh->UpdateTransforms();
	}

	void Scene_W4_ReferenceScene::Initialize()
	{
		sceneName = "Reference Scene";
		m_Camera.origin = { 0, 3, -9 };
		m_Camera.fovAngle = 45.f;

		const auto matCT_GrayRoughMetal{ AddMaterial(new Material_CookTorrence({.972f, .960f, .915f}, 1.f, 1.f)) };
		const auto matCT_GrayMediumMetal{ AddMaterial(new Material_CookTorrence({.972f, .960f, .915f}, 1.f, .6f)) };
		const auto matCT_GraySmoothMetal{ AddMaterial(new Material_CookTorrence({.972f, .960f, .915f}, 1.f, .1f)) };
		const auto matCT_GrayRoughPlastic{ AddMaterial(new Material_CookTorrence({.75f, .75f, .75f}, .0f, 1.f)) };
		const auto matCT_GrayMediumPlastic{ AddMaterial(new Material_CookTorrence({.75f, .75f, .75f}, .0f, .6f)) };
		const auto matCT_GraySmoothPlastic{ AddMaterial(new Material_CookTorrence({.75f, .75f, .75f}, .0f, .1f)) };

		const auto matLambert_GrayBlue{ AddMaterial(new Material_Lambert({.49f, .57f, .57f}, 1.f)) };
		const auto matLambert_White{ AddMaterial(new Material_Lambert(colors::White, 1.f)) };

		//Planes
		AddPlane(Vector3{ 0.f, 0.f, 10.f }, Vector3{ 0.f, 0.f, -1.f }, matLambert_GrayBlue); //back
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue); //bottom
		AddPlane(Vector3{ 0.f, 10.f, 0.f }, Vector3{ 0.f, -1.f, 0.f }, matLambert_GrayBlue); //top
		AddPlane(Vector3{ 5.f, 0.f, 0.f }, Vector3{ -1.f, 0.f, 0.f }, matLambert_GrayBlue); //right
		AddPlane(Vector3{ -5.f, 0.f, 0.f }, Vector3{ 1.f, 0.f, 0.f }, matLambert_GrayBlue); //left

		//Spheres
		m_SpheresBoundingBox.grow(AddSphere(Vector3{ -1.75, 1.f, 0.f }, .75f, matCT_GrayRoughMetal));
		m_SpheresBoundingBox.grow(AddSphere(Vector3{ 0.f, 1.f, 0.f }, .75f, matCT_GrayMediumMetal));
		m_SpheresBoundingBox.grow(AddSphere(Vector3{ 1.75, 1.f, 0.f }, .75f, matCT_GraySmoothMetal));
		m_SpheresBoundingBox.grow(AddSphere(Vector3{ -1.75, 3.f, 0.f }, .75f, matCT_GrayRoughPlastic));
		m_SpheresBoundingBox.grow(AddSphere(Vector3{ 0.f, 3.f, 0.f }, .75f, matCT_GrayMediumPlastic));
		m_SpheresBoundingBox.grow(AddSphere(Vector3{ 1.75, 3.f, 0.f }, .75f, matCT_GraySmoothPlastic));

		//TriangleMesh
		const Triangle baseTriangle = { Vector3(-0.75f, 1.5f, 0.0f), Vector3(0.75f, 0.0f, 0.0f), Vector3(-0.75f, 0.0f, 0.0f) };

		m_pMeshes[0] = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		m_pMeshes[0]->AppendTriangle(baseTriangle, true);
		m_pMeshes[0]->Translate({ -1.75f, 4.5f, 0.0f });
		m_pMeshes[0]->UpdateTransforms();

		m_pMeshes[0]->InitBVH();

		m_pMeshes[1] = AddTriangleMesh(TriangleCullMode::FrontFaceCulling, matLambert_White);
		m_pMeshes[1]->AppendTriangle(baseTriangle, true);
		m_pMeshes[1]->Translate({ 0.0f, 4.5f, 0.0f });
		m_pMeshes[1]->UpdateTransforms();

		m_pMeshes[1]->InitBVH();

		m_pMeshes[2] = AddTriangleMesh(TriangleCullMode::NoCulling, matLambert_White);
		m_pMeshes[2]->AppendTriangle(baseTriangle, true);
		m_pMeshes[2]->Translate({ 1.75f, 4.5f, 0.0f });
		m_pMeshes[2]->UpdateTransforms();

		m_pMeshes[2]->InitBVH();

		//Lights
		AddPointLight(Vector3{ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, .61f, .45f }); //Back Light
		AddPointLight(Vector3{ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, .8f, .45f }); //Front Left Light
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, .47f, .68f });
	}

	void Scene_W4_ReferenceScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		const float yawAngle{ (cos(pTimer->GetTotal()) + 1.f) / 2.f * PI_2 };

		m_TrianglesBoundingBox.min = Vector3{ INFINITY,INFINITY,INFINITY };
		m_TrianglesBoundingBox.max = -m_TrianglesBoundingBox.min;

		for (const auto pMesh : m_pMeshes)
		{
			pMesh->RotateY(yawAngle);
			pMesh->UpdateTransforms();
			BVHNode& node = pMesh->bvhNodes[pMesh->rootNodeIndex];

			m_TrianglesBoundingBox.grow(node.minAABB);
			m_TrianglesBoundingBox.grow(node.maxAABB);
		}
	}

	void Scene_W4_BunnyScene::Initialize()
	{
		m_Camera.origin = { 0, 3, -9 };
		m_Camera.fovAngle = 45.f;

		//Material
		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, .57f, .57f }, 1.f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//planes
		AddPlane(Vector3{ 0.f, 0.f, 10.f }, Vector3{ 0.f, 0.f, -1.f }, matLambert_GrayBlue); //back
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue); //bottom
		AddPlane(Vector3{ 0.f, 10.f, 0.f }, Vector3{ 0.f, -1.f, 0.f }, matLambert_GrayBlue); //top
		AddPlane(Vector3{ 5.f, 0.f, 0.f }, Vector3{ -1.f, 0.f, 0.f }, matLambert_GrayBlue); //right
		AddPlane(Vector3{ -5.f, 0.f, 0.f }, Vector3{ 1.f, 0.f, 0.f }, matLambert_GrayBlue); //left

		//Bunny Mesh
		m_pMesh = AddTriangleMesh(dae::TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/lowpoly_bunny2.obj", m_pMesh->positions, m_pMesh->normals, m_pMesh->indices);

		m_pMesh->Scale({ 2.f,2.f,2.f });

		m_pMesh->UpdateTransforms();
		m_pMesh->InitBVH();

		//Lights
		AddPointLight(Vector3{ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, .61f, .45f }); //Back Light
		AddPointLight(Vector3{ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, .8f, .45f }); //Front Left Light
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, .47f, .68f });
	}

	void Scene_W4_BunnyScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		m_pMesh->RotateY((cos(pTimer->GetTotal()) + 1.f) / 2.f * PI_2);
		m_pMesh->UpdateTransforms();

		BVHNode& node = m_pMesh->bvhNodes[m_pMesh->rootNodeIndex];
		m_TrianglesBoundingBox.grow(node.minAABB);
		m_TrianglesBoundingBox.grow(node.maxAABB);

	}

	void Scene_W4_CarScene::Initialize()
	{
		m_Camera.origin = { 0, 3, -9 };
		m_Camera.fovAngle = 45.f;

		//Material
		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, .57f, .57f }, 1.f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//planes
		AddPlane(Vector3{ 0.f, 0.f, 10.f }, Vector3{ 0.f, 0.f, -1.f }, matLambert_GrayBlue); //back
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue); //bottom
		AddPlane(Vector3{ 0.f, 10.f, 0.f }, Vector3{ 0.f, -1.f, 0.f }, matLambert_GrayBlue); //top
		AddPlane(Vector3{ 5.f, 0.f, 0.f }, Vector3{ -1.f, 0.f, 0.f }, matLambert_GrayBlue); //right
		AddPlane(Vector3{ -5.f, 0.f, 0.f }, Vector3{ 1.f, 0.f, 0.f }, matLambert_GrayBlue); //left

		//Bunny Mesh
		m_pMesh = AddTriangleMesh(dae::TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/car.obj", m_pMesh->positions, m_pMesh->normals, m_pMesh->indices);

		//m_pMesh->Scale({ 2.f,2.f,2.f });

		m_pMesh->UpdateTransforms();
		m_pMesh->InitBVH();

		//Lights
		AddPointLight(Vector3{ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, .61f, .45f }); //Back Light
		AddPointLight(Vector3{ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, .8f, .45f }); //Front Left Light
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, .47f, .68f });
	}

	void Scene_W4_CarScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		//m_pMesh->RotateY((cos(pTimer->GetTotal()) + 1.f) / 2.f * PI_2);
		//m_pMesh->UpdateTransforms();

		BVHNode& node = m_pMesh->bvhNodes[m_pMesh->rootNodeIndex];
		m_TrianglesBoundingBox.grow(node.minAABB);
		m_TrianglesBoundingBox.grow(node.maxAABB);

	}
#pragma endregion
}
