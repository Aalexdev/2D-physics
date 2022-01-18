#pragma once

/**
 * @warning this file will change of location
 */

// libs
#include <glm/glm.hpp>


class Line2D{
	public:
		Line2D(){}
		Line2D(glm::vec2 from, glm::vec2 to) : start{from}, end{to} {}
		
		/**
		 * @brief get the line start
		 * @return glm::vec2 
		 */
		glm::vec2 getStart() const noexcept {return start;}

		/**
		 * @brief get the line end
		 * @return glm::vec2 
		 */
		glm::vec2 getEnd() const noexcept {return end;}


	private:
		glm::vec2 start, end;
};