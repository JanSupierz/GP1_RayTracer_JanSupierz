//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

#include <thread>
#include <future> //Async
#include <ppl.h>

using namespace dae;

//#define ASYNC
#define PARALLEL_FOR

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
	m_AspectRatio = static_cast<float>(m_Width) / static_cast<float>(m_Height);
	m_NumberOfPixels = m_Width * m_Height;
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const float fieldOfView{ tanf(camera.fovAngle*TO_RADIANS / 2.f) };

#if defined(ASYNC)
	//Async logic
	const uint32_t numCores = std::thread::hardware_concurrency();
	std::vector<std::future<void>> async_futures{};

	const uint32_t numPixelsPerTask = numPixels / numCores;
	uint32_t numUnassignedPixels = numPixels % numCores;
	uint32_t currentPixelIndex{ 0 };

	for (uint32_t coreId{ 0 }; coreId < numCores; ++coreId)
	{
		uint32_t taskSize{ numPixelsPerTask };
		if (numUnassignedPixels > 0)
		{
			++taskSize;
			--numUnassignedPixels;
		}

		async_futures.push_back(std::async(std::launch::async, [=, this]
			{
				const uint32_t pixelIndexEnd = currentPixelIndex + taskSize;
				for (uint32_t pixelIndex{ currentPixelIndex }; pixelIndex < pixelIndexEnd; ++pixelIndex)
				{
					RenderPixel(pScene, pixelIndex, fieldOfView, camera, lights, materials);
				}
				
			})
		);

		currentPixelIndex += taskSize;
	}

	//Wait for all tasks
	for (const std::future<void>& f : async_futures)
	{
		f.wait();
	}

#elif defined(PARALLEL_FOR)
	//Parallel For logic
	concurrency::parallel_for(0u, m_NumberOfPixels, [=, this](int i)
	{
			RenderPixel(pScene, i, fieldOfView, camera, lights, materials);
	});

#else
	//Synchronous logic
	for (uint32_t i{ 0 }; i < m_NumberOfPixels; ++i)
	{
		RenderPixel(pScene, i, fieldOfView, camera, lights, materials);
	}
#endif

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::RenderPixel(Scene* pScene, uint32_t pixelIndex, float fieldOfView, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const
{
	const uint32_t px{ pixelIndex % m_Width }, py{ pixelIndex / m_Width };

	Ray viewRay{ camera.origin ,Vector3::Zero };
	ColorRGB finalColor{ dae::colors::Black };

	const float cx{ (((2.f * (px + 0.5f)) / static_cast<float>(m_Width)) - 1) * m_AspectRatio * fieldOfView };
	const float cy{ (1 - ((2.f * (py + 0.5f)) / static_cast<float>(m_Height))) * fieldOfView };

	viewRay.direction = (cx * Vector3::UnitX) + (cy * Vector3::UnitY) + Vector3::UnitZ;
	viewRay.direction.Normalize();
	viewRay.direction = camera.cameraToWorld.TransformVector(viewRay.direction);

	HitRecord closestHit{};
	pScene->GetClosestHit(viewRay, closestHit);

	if (closestHit.didHit)
	{
		Ray lightRay{ closestHit.origin + closestHit.normal * 0.0002f };

		for (const Light& light : lights)
		{
			lightRay.direction = LightUtils::GetDirectionToLight(light, lightRay.origin);
			lightRay.max = lightRay.direction.Normalize();

			if (m_ShadowsEnabled) //als de schaduwen aan staan
			{
				if (!pScene->DoesHit(lightRay)) //en als er niets zit tussen de lichtbron en deze pixel
				{
					CalculateFinalColor(light, lightRay.direction, closestHit, materials, viewRay.direction, finalColor); //dan berekenen we licht
				}
			}
			else
			{
				CalculateFinalColor(light, lightRay.direction, closestHit, materials, viewRay.direction, finalColor);
			}
		}
	}

	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	if (m_CurrentLightingMode != LightingMode::Combined)
	{
		m_CurrentLightingMode = static_cast<LightingMode>(static_cast<int>(m_CurrentLightingMode) + 1);
	}
	else
	{
		m_CurrentLightingMode = LightingMode::ObservedArea;
	}
}

void Renderer::CalculateFinalColor(const Light& light, const Vector3& lightRayDirection, const HitRecord& closestHit, const std::vector<Material*>& materials, const Vector3& viewRayDirection, ColorRGB& finalColor) const
{
	float observedArea{ Vector3::Dot(closestHit.normal,lightRayDirection) };

		switch (m_CurrentLightingMode)
		{

		case LightingMode::ObservedArea:
			if (observedArea > 0.f)
			{
				finalColor += ColorRGB{ 1.f,1.f,1.f } * observedArea;
			}
			break;

		case LightingMode::Radiance:
			finalColor += LightUtils::GetRadiance(light, closestHit.origin);
			break;

		case LightingMode::BRDF:
				finalColor += materials[closestHit.materialIndex]->Shade(closestHit, lightRayDirection, viewRayDirection);
			break;

		case LightingMode::Combined:
			if (observedArea > 0.f)
			{
				finalColor += LightUtils::GetRadiance(light, closestHit.origin) * materials[closestHit.materialIndex]->Shade(closestHit, lightRayDirection, viewRayDirection) * observedArea;
			}
			break;
		}
}