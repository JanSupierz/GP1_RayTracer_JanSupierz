#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

#include <iostream>

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle) :
			origin{ _origin },
			fovAngle{ _fovAngle }
		{
		}

		bool hasMoved{ false };

		Vector3 origin{};
		float fovAngle{ 90.f };

		Vector3 forward{ Vector3::UnitZ };
		Vector3 up{ Vector3::UnitY };
		Vector3 right{ Vector3::UnitX };

		float totalPitch{ 0.f };
		float totalYaw{ 0.f };

		Matrix cameraToWorld;

		Matrix CalculateCameraToWorld()
		{
			if (hasMoved)
			{
				Matrix rotationMatrix{ Matrix::CreateRotation(totalPitch,totalYaw,0.f) };

				forward = rotationMatrix.TransformVector(Vector3::UnitZ);
				forward.Normalize();

				right = Vector3::Cross(Vector3::UnitY, forward).Normalized();
				up = Vector3::Cross(forward, right).Normalized();

				hasMoved = false;

				return { right,up,forward,origin };
			}

			return cameraToWorld;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			float movementSpeed{ 5.f };
			float rotationSpeed{ 0.5f };

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			if (pKeyboardState[SDL_SCANCODE_LSHIFT])
			{
				const float factor{ 4.f };

				movementSpeed *= factor;
				rotationSpeed *= factor;
			}

			if (pKeyboardState[SDL_SCANCODE_W])
			{
				origin += forward * movementSpeed * deltaTime;
				hasMoved = true;
			}

			if (pKeyboardState[SDL_SCANCODE_S]) 
			{
				origin -= forward * movementSpeed * deltaTime;
				hasMoved = true;
			}

			if (pKeyboardState[SDL_SCANCODE_D]) 
			{
				origin += right * movementSpeed * deltaTime;
				hasMoved = true;
			}

			if (pKeyboardState[SDL_SCANCODE_A])
			{
				origin -= right * movementSpeed * deltaTime;
				hasMoved = true;
			}

			if (pKeyboardState[SDL_SCANCODE_LEFT])
			{
				fovAngle -= movementSpeed * deltaTime;
				fovAngle = std::max(fovAngle, 1.f);
			}

			if (pKeyboardState[SDL_SCANCODE_RIGHT])
			{
				fovAngle += movementSpeed * deltaTime;
				fovAngle = std::min(fovAngle, 179.f);
			}

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			if (mouseX != 0 || mouseY != 0)
			{
				const float directionX{ static_cast<float>(mouseX) }, directionY{ static_cast<float>(mouseY) };

				if ((mouseState & SDL_BUTTON_LMASK) && (mouseState & SDL_BUTTON_RMASK))
				{
					origin += right * movementSpeed * directionX * deltaTime;
					origin -= up * movementSpeed * directionY * deltaTime;
					hasMoved = true;
				}
				else if (mouseState & SDL_BUTTON_LMASK)
				{
					totalYaw += rotationSpeed * directionX * deltaTime;
					origin -= forward * movementSpeed * directionY * deltaTime;
					hasMoved = true;
				}
				else if (mouseState & SDL_BUTTON_RMASK)
				{
					totalYaw += rotationSpeed * directionX * deltaTime;
					totalPitch -= rotationSpeed * directionY * deltaTime;
					hasMoved = true;
				}
				else if (mouseState)
				{
					origin -= forward * movementSpeed * directionY * deltaTime;
					hasMoved = true;
				}
			}

			cameraToWorld = CalculateCameraToWorld();
		}
	};
}
