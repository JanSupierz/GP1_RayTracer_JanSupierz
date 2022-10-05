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

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const float fieldOfView{ tanf(camera.fovAngle*TO_RADIANS / 2.f) };
	const float aspectRatio{ static_cast<float>(m_Width) / static_cast<float>(m_Height) };

	Ray viewRay{ camera.origin ,Vector3::Zero };
	ColorRGB finalColor{};

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			const float cx{ (((2.f * (px + 0.5f)) / static_cast<float>(m_Width)) - 1) * aspectRatio * fieldOfView };
			const float cy{ (1 - ((2.f * (py + 0.5f)) / static_cast<float>(m_Height))) * fieldOfView };

			viewRay.direction = (cx * Vector3::UnitX) + (cy * Vector3::UnitY) + Vector3::UnitZ;
			viewRay.direction.Normalize();
			viewRay.direction = camera.cameraToWorld.TransformVector(viewRay.direction);

			
			HitRecord closestHit{};
			pScene->GetClosestHit(viewRay, closestHit);

			finalColor = dae::colors::Black;

			if (closestHit.didHit)
			{
				finalColor = materials[closestHit.materialIndex]->Shade();
	

				for (const Light& light : pScene->GetLights())
				{
					Ray lightRay{};
					lightRay.origin = closestHit.origin + closestHit.normal * lightRay.min;
					lightRay.direction = LightUtils::GetDirectionToLight(light, lightRay.origin);
					lightRay.max = lightRay.direction.Normalize();

					if (pScene->DoesHit(lightRay))
					{
						finalColor *= 0.5f;
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
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}
