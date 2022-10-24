#pragma once

#include <cstdint>
#include <vector>

struct SDL_Window;
struct SDL_Surface;


namespace dae
{
	struct Light;
	struct Vector3;
	struct HitRecord;
	struct ColorRGB;

	class Material;

	class Scene;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer() = default;

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Render(Scene* pScene) const;
		bool SaveBufferToImage() const;
		
		void CycleLightingMode();
		void ToggleShadows() { m_ShadowsEnabled = !m_ShadowsEnabled; };

	private:
		SDL_Window* m_pWindow{};

		SDL_Surface* m_pBuffer{};
		uint32_t* m_pBufferPixels{};

		int m_Width{};
		int m_Height{};

		enum class LightingMode
		{
			ObservedArea, //Lambert cosine law
			Radiance, //Incident radiance
			BRDF, //Scattering of the light
			//WithoutShade,
			Combined
		};

		LightingMode m_CurrentLightingMode{ LightingMode::Combined };
		bool m_ShadowsEnabled{ true };

		void CalculateFinalColor(const Light& light, const Vector3& lightRayDirection, const HitRecord& closestHit, const std::vector<Material*>& materials, const Vector3& viewRayDirection, ColorRGB& finalColor) const;
	};
}
