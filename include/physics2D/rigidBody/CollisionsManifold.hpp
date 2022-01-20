#pragma once

// libs
#include <glm/glm.hpp>

// std
#include <vector>

namespace physics2D::rigidBody{
    class CollisionsManifold{
        public:
            CollisionsManifold() {}
            CollisionsManifold(glm::vec2 normal, float depth) : normal{normal}, depth{depth} {}

            /**
             * @brief get the normal of the collisions
             * @return glm::vec2 
             */
            glm::vec2 getNormal() const noexcept {return normal;}

            /**
             * @brief Get the Contact Points object
             * @note return a referenceto avoid usless copy and time wasting
             * @return std::vector<glm::vec2>& 
             */
            std::vector<glm::vec2>& getContactPoints() noexcept {return contactPoints;}

            /**
             * @brief get the depth of the collision
             * @return int 
             */
            int getDept() const noexcept {return depth;}

            /**
             * @brief set the normal of the collision
             * @param normal 
             */
            void setNormal(glm::vec2 normal) noexcept {this->normal = normal;}

            /**
             * @brief set the depth of the collision
             * @param newDepth 
             */
            void setDeth(float newDepth) {depth = newDepth;}

            /**
             * @brief get if there is a collision
             * @return true 
             * @return false 
             */
            bool isColliding() const noexcept {return colliding;}

            /**
             * @brief of ther is any collision
             * @param colliding 
             */
            void setColliding(bool colliding) {this->colliding = colliding;}

        private:
            glm::vec2 normal; 
            std::vector<glm::vec2> contactPoints;
            float depth = 0.f;
            bool colliding = false;
    };
}